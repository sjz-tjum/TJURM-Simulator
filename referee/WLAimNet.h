// WLAimNet.h
//
// 自瞄闭环的 UDP 桥（用户 2026-10-10：「做一下接入」）。
//
// 角色划分：**模拟器扮演"车 + 相机 + 下位机"**，自瞄程序（他们的 Python/C++）只做识别与解算。
// 三个端口，localhost：
//   41001  sim -> 自瞄   相机帧（JPEG，分片）
//   41002  sim -> 自瞄   云台状态（yaw/pitch/roll + 时间戳）
//   41003  自瞄 -> sim   云台增量指令（dYaw/dPitch 度 + 开火）
//
// 为什么用"增量"而不是绝对角：自瞄端只看得到像素，不知道仿真的世界朝向；视觉伺服本来就是
// 误差 -> 增量。仿真端把增量加到 Gimbal.YawTarget/PitchTarget 上并做限位。
//
// 帧分片头（20 字节，小端）：
//   uint32 Magic('RMAI') uint32 FrameId uint16 ChunkIdx uint16 ChunkCount
//   uint32 TotalBytes   uint32 PayloadBytes
// 状态包（24 字节）：uint32 Magic('RMST') float yawDeg float pitchDeg float rollDeg float timeSec
// 指令包（24 字节）：uint32 Magic('RMCM') float dYawDeg float dPitchDeg uint8 fire uint8 pad[3] float timeSec

#pragma once

#include "CoreMinimal.h"

class FSocket;
class FInternetAddr;

/** 非 UObject 的小工具类：pawn 持有一个 TUniquePtr，不需要 UHT。 */
struct RM2019SIM_API FWLAimNet
{
	/** 相机在**仿真世界系**下的位姿（米 / 度），跟着帧一起发。
	 *
	 *  为什么要它（2026-10-12 定案）：绝对角通道要的是"目标相对**这张图的相机**的方位"，
	 *  所以必须用**拍摄这一刻**的相机姿态。原来图像走 41001、位姿走 41002，是两个包、
	 *  两条队列，peer 又只保留最后一个状态包 ⇒ 一帧图像配上的是**几十毫秒之后**的姿态。
	 *  云台一转，整帧方位就偏几十度（实测 cam 恒定、而 PnP 方位与像素方位差 66° ✓），
	 *  闭环于是变成"越偏越转"。把位姿塞进**同一个 UDP 包**就没有这个时间差 ✓。
	 *
	 *  Dx/Dy/Dz（2026-10-12 追加）：**相机 → 枪口**的偏移（仿真世界系，米）。
	 *  自瞄相机有一点安装偏置（实测 cam=(799.4,600.8,64.4) vs muzzle=(800.0,600.0,64.5) cm
	 *  ⇒ 相机比枪口**低约 1 cm** 且横向偏 0.75 cm）。用相机位姿算出来的瞄准线因此与
	 *  实际弹道**不平行**：视觉瞄准了板心，炮口却偏上/偏一侧（用户实测"整体偏右、pitch 偏上"✓）。
	 *  带上这个偏移，peer 就能把瞄准线从"相机→目标"换成真正的"**枪口→目标**" ✓ */
	struct FCamPose
	{
		float Px, Py, Pz;              // 相机光心（仿真世界系，米）
		float Yaw, Pitch, Roll;        // 相机朝向（度；yaw 正=左、pitch 正=抬头、roll 与仿真一致）
		float Dx, Dy, Dz;              // 相机 -> 枪口 的偏移（仿真世界系，米）
	};

	bool Start(int32 FramePort = 41001, int32 StatePort = 41002, int32 CmdPort = 41003);
	void Stop();

	/** 发一帧 JPEG（自动分片）。
	 *  第 0 片在 JPEG 之后**追加 24 字节**相机位姿（6 个 float），
	 *  这样"图像"和"拍它的相机姿态"在同一个包里，peer 解码这一帧时拿到的就是同一时刻的姿态。
	 *  旧 peer 按 PayloadBytes 只读 JPEG、多余字节自然忽略 ⇒ 向后兼容 ✓。 */
	bool SendFrame(uint32 FrameId, const TArray<uint8>& Jpeg, const FCamPose* CamPose = nullptr);
	/** 发云台状态 + 目标距离 + **世界系位姿**。
	 *  RangeCm：到目标（画面正中那台车）的距离，用来做弹道补偿。
	 *  Poses18：18 个 float —— chassis(pos,rpy), gimbal(pos,rpy), muzzle(pos), camera(rpy)，
	 *  **厘米/度**（仿真内部单位；视觉端按他们的契约换算成米/弧度）。
	 *  为什么需要：PlugYourRM 的 SelfState 要世界系绝对位姿 + 内参 K，PnP 才能解出目标位置。 */
	void SendState(float YawDeg, float PitchDeg, float RollDeg, double TimeSec, float RangeCm,
		const float* Poses18 = nullptr);
	/** 收指令；拿到一条就返回 true。
	 *  bOutHasTarget：视觉端本帧是否真的识别到目标（包内第 13 字节，原来是填充位，包长不变）。
	 *  仿真用它决定"自瞄状态下要不要屏蔽鼠标"：有目标时自瞄全权接管，没目标时操作手还能手动转 ✗。
	 *  ---- 绝对角扩展（2026-10-11，为了接 PlugYourRM 那套"位置指令"的自瞄）----
	 *  偏移 20: float AbsYawDeg（绝对世界朝向，他们的 yaw 逆时针为正）
	 *  偏移 24: float AbsPitchDeg（抬头为正的仰角）
	 *  偏移 28: uint8 UseAbsolute（1 = 用绝对角，忽略前面那对增量）
	 *  为什么要它：他们的 ShotSolution 是**绝对角度**，做闭环位置控制；把它差分成增量喂速度接口
	 *  等于对误差求导 → 噪声放大，且同一帧重复 tick 时增量为 0 → 表现就是"yaw 不跟 + 越远越晃"。 */
	bool PollCommand(float& OutDYawDeg, float& OutDPitchDeg, bool& bOutFire, bool& bOutHasTarget,
		float& OutAbsYawDeg, float& OutAbsPitchDeg, bool& bOutUseAbsolute, uint8* OutAbsCode = nullptr);

	/** 自愈看门狗（每秒调一次）。Windows 上 UDP 两侧都会被 ICMP 毒化：实测视觉端一重启，
	 *  仿真的 sendto 就**永久失败**（日志里 sent 计数冻住、视觉端 WAITING、cmds 恒 0 ✗），
	 *  接收方向同理。这里在"持续失败 / 长时间收不到"时重建 socket，
	 *  免得每次重启视觉端都要人工按两次 O。 */
	void Watchdog(double NowSec);

	/** 视觉端回传的检测结果（用于在自瞄画面里把"识别到的灯条/四角点"画出来，
	 *  用户 2026-10-11 明确要求：看不到识别结果就没法判断链路对不对）。
	 *  包格式：uint32 magic('RMDT') uint16 count，随后每条 4 个 float2（LT/RT/RB/LB，图像像素，
	 *  以 1280x1024 为准）。端口 41004，独立通道，不影响控制。 */
	void PollDetections();
	/** 取回传的检测框（每条 4 个点，图像像素坐标）。 */
	const TArray<FVector2D>& GetDetections() const { return Detections; }
	int32 DetCount = 0;
	/** 回传的检测对应的**帧号**：HUD 只画"与当前显示帧一致（±2 帧）"的检测。
	 *  云台一动，屏幕上的图是更早那帧、框却是最新那帧 ⇒ 框会跟着晃（用户实测 ✗）；
	 *  用帧号对齐就钉住了。 */
	int32 DetFrameId = -1;

	/** 每一帧"发出时"的云台世界角（yaw 弧度 / 仰角弧度），按帧号索引。
	 *  用途（用户反馈"绿框跟着云台飘"✓）：检测天然比画面晚 3~5 帧，画框时用
	 *  "检测帧 -> 现在"的转角差把点平移回来 ⇒ 板不动时框就钉住。
	 *  像素换算：du = -f*dyaw，dv = +f*dpitch（f=1250，见 AimCamFovDeg）。 */
	struct FAimFrameSample { int32 Id; float YawRad; float PitchRad; };
	void NoteFrameSample(int32 Id, float YawRad, float PitchRad);
	bool GetFrameSample(int32 Id, float& OutYawRad, float& OutPitchRad) const;
	TArray<FAimFrameSample> FrameRing;

	int32 SendFails = 0;         // 连续发送失败次数
	double LastDetRecvTime = 0.0;    // 上次收到检测回传的时刻（HUD 用它显示"数据有多新"）
	double NowDetLogSeconds = 0.0;   // 回传日志节流
	double LastDetRebuildTime = 0.0; // 41004 定期重建的记时
	int32 SendRebuilds = 0;      // 重建发送 socket 次数
	int32 RecvRebuilds = 0;      // 重建接收 socket 次数
	double LastCfgSendTime = -1.0;   // 上次"有发无收"的记时基准

	// 统计（日志用）
	int32 FramesSent = 0;
	int32 PacketsSent = 0;
	int32 CmdsRecv = 0;
	int64 BytesSent = 0;
	int32 LastFrameId = 0;
	bool bStarted = false;

private:
	FSocket* SendSock = nullptr;
	FSocket* RecvSock = nullptr;
	FSocket* DetSock = nullptr;          // 41004：视觉端回传的检测结果
	TArray<FVector2D> Detections;        // 每条 4 点，展平存放
	TSharedPtr<FInternetAddr> FrameAddr;
	TSharedPtr<FInternetAddr> StateAddr;
};
