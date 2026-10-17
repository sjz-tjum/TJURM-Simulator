// WLAimNet.cpp —— 自瞄 UDP 桥的实现。协议说明见 WLAimNet.h。

#include "WLAimNet.h"

#include "Sockets.h"
#include "SocketSubsystem.h"
// 注意：不要 include "Interfaces/IPv4/IPv4Address.h" —— 那个头属于 Networking 模块，
// 本模块只依赖 Sockets，加进去会 C1083（实测）。用 SS->CreateInternetAddr() 就够了。

namespace
{
	constexpr uint32 MagicFrame = 0x49414D52;      // 'RMAI' little-endian
	constexpr uint32 MagicState = 0x54534D52;      // 'RMST'
	constexpr uint32 MagicCmd   = 0x4D434D52;      // 'RMCM'
	/** 每个分片的净荷上限：UDP 下 1200 字节足够安全（避免 IP 分片再分片）。 */
	constexpr int32 ChunkPayload = 1200;
	/** 一次性接收缓冲：够一条指令包就好。 */
	constexpr int32 RecvBufSize = 256;
}

bool FWLAimNet::Start(int32 FramePort, int32 StatePort, int32 CmdPort)
{
	ISocketSubsystem* SS = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
	if (!SS)
	{
		UE_LOG(LogTemp, Warning, TEXT("WLAIMNET|no socket subsystem"));
		return false;
	}
	SendSock = SS->CreateSocket(NAME_DGram, TEXT("WLAimSend"));
	if (!SendSock)
	{
		UE_LOG(LogTemp, Warning, TEXT("WLAIMNET|could not create the send socket"));
		return false;
	}
	SendSock->SetNonBlocking(true);
	SendSock->SetReuseAddr(true);
	FrameAddr = SS->CreateInternetAddr();
	StateAddr = SS->CreateInternetAddr();
	bool bOk = false;
	FrameAddr->SetIp(TEXT("127.0.0.1"), bOk);
	FrameAddr->SetPort(FramePort);
	StateAddr->SetIp(TEXT("127.0.0.1"), bOk);
	StateAddr->SetPort(StatePort);

	RecvSock = SS->CreateSocket(NAME_DGram, TEXT("WLAimRecv"));
	if (RecvSock)
	{
		RecvSock->SetNonBlocking(true);
		RecvSock->SetReuseAddr(true);
		TSharedRef<FInternetAddr> Bind = SS->CreateInternetAddr();
		Bind->SetIp(TEXT("127.0.0.1"), bOk);
		Bind->SetPort(CmdPort);
		if (!RecvSock->Bind(*Bind))
		{
			// ★ 这里以前只是 Warning，然后照样 bStarted=true → 假"桥起来了"：
			// 表现就是"帧照发、cmds 永远 0"。实测根因是上一次 PIE 的 socket 被泄漏
			// （pawn 销毁时没关），编辑器进程一直占着 41003。现在直接判失败，前端会显示 link=DOWN。
			UE_LOG(LogTemp, Error,
				TEXT("WLAIMNET|BIND FAILED on %d - another bridge still holds it. Close/restart the editor once, or press O twice."),
				CmdPort);
			RecvSock->Close();
			SS->DestroySocket(RecvSock);
			RecvSock = nullptr;
			Stop();
			return false;
		}
		// 收缓冲调大一点，避免高帧率指令被丢
		int32 NewSize = 0;
		RecvSock->SetReceiveBufferSize(1 << 20, NewSize);
	}
	else
	{
		UE_LOG(LogTemp, Error, TEXT("WLAIMNET|could not create the command socket"));
		Stop();
		return false;
	}

	bStarted = true;
	UE_LOG(LogTemp, Display,
		TEXT("WLAIMNET|up: frames->%d, state->%d, commands<-%d (JPEG chunk %d B)"),
		FramePort, StatePort, CmdPort, ChunkPayload);
	return true;
}

void FWLAimNet::Stop()
{
	if (SendSock)
	{
		SendSock->Close();
		ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(SendSock);
		SendSock = nullptr;
	}
	if (RecvSock)
	{
		RecvSock->Close();
		ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(RecvSock);
		RecvSock = nullptr;
	}
	// ★★ 关键修复（2026-10-11，用户"重启多少次都不显示绿色角点"的真正原因）：
	// 以前这里漏掉了检测回传通道 41004 的 socket ⇒ **每次 Play 泄漏一个** ⇒
	// Windows 同端口多 socket 只有一个能收 ⇒ 旧 socket 把回传全收走 ⇒
	// 当前这局的 Detections 永远为空 ⇒ HUD 永远画不出角点 ✗✗。
	// 实测证据：41004 上出现**两个**属于编辑器的 socket ✓✓。
	if (DetSock)
	{
		DetSock->Close();
		ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(DetSock);
		DetSock = nullptr;
	}
	bStarted = false;
}

bool FWLAimNet::SendFrame(uint32 FrameId, const TArray<uint8>& Jpeg, const FCamPose* CamPose)
{
	if (!SendSock || !FrameAddr.IsValid() || Jpeg.Num() == 0)
	{
		++SendFails;
		return false;
	}
	const int32 Total = Jpeg.Num();
	const int32 ChunkCount = (Total + ChunkPayload - 1) / ChunkPayload;
	const int32 PoseBytes = CamPose ? (int32)sizeof(FCamPose) : 0;   // 36 字节，只挂第 0 片
	TArray<uint8> Packet;
	Packet.Reserve(ChunkPayload + 20 + PoseBytes);
	for (int32 c = 0; c < ChunkCount; ++c)
	{
		const int32 Offset = c * ChunkPayload;
		const int32 Len = FMath::Min(ChunkPayload, Total - Offset);
		Packet.Reset();
		auto Push32 = [&Packet](uint32 V) { Packet.Append((uint8*)&V, 4); };
		auto Push16 = [&Packet](uint16 V) { Packet.Append((uint8*)&V, 2); };
		Push32(MagicFrame);
		Push32(FrameId);
		Push16((uint16)c);
		Push16((uint16)ChunkCount);
		Push32((uint32)Total);
		Push32((uint32)Len);
		Packet.Append(Jpeg.GetData() + Offset, Len);
		// 第 0 片后面追加相机位姿：图像与"拍它的相机姿态"同包 ⇒ 无时间差 ✓
		if (c == 0 && CamPose)
		{
			Packet.Append((const uint8*)CamPose, PoseBytes);
		}
		int32 Sent = 0;
		if (!SendSock->SendTo(Packet.GetData(), Packet.Num(), Sent, *FrameAddr))
		{
			// Windows: 对端消失后 sendto 会**持续**失败（ICMP 毒化）——交给 Watchdog 重建 socket
			++SendFails;
			return false;
		}
		SendFails = 0;
		++PacketsSent;
		BytesSent += Sent;
	}
	++FramesSent;
	LastFrameId = (int32)FrameId;
	return true;
}

void FWLAimNet::SendState(float YawDeg, float PitchDeg, float RollDeg, double TimeSec, float RangeCm,
	const float* Poses18)
{
	if (!SendSock || !StateAddr.IsValid())
	{
		return;
	}
	// 24 字节基础包 + 可选 72 字节世界系位姿（18 个 float）。视觉端按长度判断有没有扩展段。
	uint8 Buf[24 + 72];
	uint32 Magic = MagicState;
	float T = (float)TimeSec;
	FMemory::Memcpy(Buf + 0, &Magic, 4);
	FMemory::Memcpy(Buf + 4, &YawDeg, 4);
	FMemory::Memcpy(Buf + 8, &PitchDeg, 4);
	FMemory::Memcpy(Buf + 12, &RollDeg, 4);
	FMemory::Memcpy(Buf + 16, &T, 4);
	// 偏移 20 是到目标的距离（给弹道补偿用）
	FMemory::Memcpy(Buf + 20, &RangeCm, 4);
	int32 Len = 24;
	if (Poses18)
	{
		FMemory::Memcpy(Buf + 24, Poses18, 18 * sizeof(float));
		Len = 24 + 72;
	}
	int32 Sent = 0;
	SendSock->SendTo(Buf, Len, Sent, *StateAddr);
	BytesSent += Sent;
}

// ------------------------------------------------------------------ 自愈
void FWLAimNet::NoteFrameSample(int32 Id, float YawRad, float PitchRad)
{
	FrameRing.Add(FAimFrameSample{ Id, YawRad, PitchRad });
	if (FrameRing.Num() > 64)
	{
		FrameRing.RemoveAt(0, FrameRing.Num() - 64, false);
	}
}

bool FWLAimNet::GetFrameSample(int32 Id, float& OutYawRad, float& OutPitchRad) const
{
	for (int32 i = FrameRing.Num() - 1; i >= 0; --i)
	{
		if (FrameRing[i].Id == Id)
		{
			OutYawRad = FrameRing[i].YawRad;
			OutPitchRad = FrameRing[i].PitchRad;
			return true;
		}
	}
	return false;
}

void FWLAimNet::Watchdog(double NowSec)
{
	ISocketSubsystem* SS = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
	if (!SS)
	{
		return;
	}
	// (1) 发送侧：连续失败（典型 1 秒的量级）就重建 socket + 重新解析目标地址。
	//     实测：视觉端一重启，ICMP 会让 sendto 永久失败，日志里 sent 计数冻住、视觉端 WAITING。
	if (SendFails > 20)
	{
		if (SendSock)
		{
			SendSock->Close();
			SS->DestroySocket(SendSock);
			SendSock = nullptr;
		}
		SendSock = SS->CreateSocket(NAME_DGram, TEXT("WLAimSend"));
		if (SendSock)
		{
			SendSock->SetNonBlocking(true);
			SendSock->SetReuseAddr(true);
			SendFails = 0;
			++SendRebuilds;
			UE_LOG(LogTemp, Warning, TEXT("WLAIMNET|send socket rebuilt (#%d) - ICMP poisoning"), SendRebuilds);
		}
	}
	// (2b) 检测回传通道（41004）：同样会被 ICMP/端口抢占弄哑 —— 实测我的抓包工具
	// （SO_REUSEADDR，"最后绑定者赢"）能把仿真的 41004 顶掉，之后**绿色角点就永远不显示** ✗。
	// 这里索性每 3 秒重建一次，保证"只要视觉端在发，仿真就能收到"。
	if (NowSec - LastDetRebuildTime > 3.0)
	{
		LastDetRebuildTime = NowSec;
		if (DetSock)
		{
			ISocketSubsystem* SS2 = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
			DetSock->Close();
			if (SS2)
			{
				SS2->DestroySocket(DetSock);
			}
			DetSock = nullptr;
		}
	}

	// (2) 接收侧：有发无收超过 3 秒 ⇒ 重建（同理，收方向也会被毒化）
	if (RecvSock)
	{
		if (LastCfgSendTime < 0.0)
		{
			LastCfgSendTime = NowSec;
		}
		const bool bStale = (FramesSent > 0) && (CmdsRecv == 0) && (NowSec - LastCfgSendTime > 3.0);
		if (bStale)
		{
			RecvSock->Close();
			SS->DestroySocket(RecvSock);
			RecvSock = SS->CreateSocket(NAME_DGram, TEXT("WLAimRecv"));
			if (RecvSock)
			{
				RecvSock->SetNonBlocking(true);
				RecvSock->SetReuseAddr(true);
				TSharedRef<FInternetAddr> Bind = SS->CreateInternetAddr();
				bool bOk = false;
				Bind->SetIp(TEXT("127.0.0.1"), bOk);
				Bind->SetPort(41003);
				if (RecvSock->Bind(*Bind))
				{
					++RecvRebuilds;
					UE_LOG(LogTemp, Warning, TEXT("WLAIMNET|recv socket rebuilt (#%d) - no commands for 3 s"), RecvRebuilds);
				}
			}
			LastCfgSendTime = NowSec;
		}
	}
}

// ------------------------------------------------------------------ 检测结果回传（画在自瞄画面上）
void FWLAimNet::PollDetections()
{
	if (!DetSock)
	{
		ISocketSubsystem* SS = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
		if (!SS)
		{
			return;
		}
		DetSock = SS->CreateSocket(NAME_DGram, TEXT("WLAimDet"));
		if (!DetSock)
		{
			return;
		}
		DetSock->SetNonBlocking(true);
		DetSock->SetReuseAddr(true);
		TSharedRef<FInternetAddr> Bind = SS->CreateInternetAddr();
		bool bOk = false;
		Bind->SetIp(TEXT("127.0.0.1"), bOk);
		Bind->SetPort(41004);
		if (!DetSock->Bind(*Bind))
		{
			SS->DestroySocket(DetSock);
			DetSock = nullptr;
			return;
		}
	}
	static const uint32 MagicDet = 0x54444D52;      // 'RMDT'
	uint8 Buf[1024];
	int32 Read = 0;
	TSharedRef<FInternetAddr> From = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();
	bool bGot = false;
	while (DetSock->RecvFrom(Buf, sizeof(Buf), Read, *From) && Read >= 6)
	{
		uint32 Magic = 0;
		FMemory::Memcpy(&Magic, Buf, 4);
		if (Magic != MagicDet)
		{
			continue;
		}
		uint16 Count = 0;
		FMemory::Memcpy(&Count, Buf + 4, 2);
		const int32 Want = 6 + (int32)Count * 8 * 4;
		if (Read < Want || Count > 16)
		{
			continue;
		}
		Detections.Reset();
		for (int32 i = 0; i < (int32)Count; ++i)
		{
			for (int32 p = 0; p < 4; ++p)
			{
				float X = 0.0f, Y = 0.0f;
				FMemory::Memcpy(&X, Buf + 6 + (i * 8 + p * 2) * 4, 4);
				FMemory::Memcpy(&Y, Buf + 6 + (i * 8 + p * 2 + 1) * 4, 4);
				Detections.Add(FVector2D(X, Y));
			}
		}
		DetCount = (int32)Count;
		LastDetRecvTime = FPlatformTime::Seconds();
		bGot = true;
	}
	// 低频日志：把"有没有收到回传"变成可观测事实（否则只能靠猜 ✗）
	if (DetCount > 0 && NowDetLogSeconds < LastDetRecvTime)
	{
		NowDetLogSeconds = LastDetRecvTime + 2.0;
		UE_LOG(LogTemp, Display, TEXT("WLAIMDET|recv %d plate(s) from the vision peer"), DetCount);
	}
}

bool FWLAimNet::PollCommand(float& OutDYawDeg, float& OutDPitchDeg, bool& bOutFire, bool& bOutHasTarget,
	float& OutAbsYawDeg, float& OutAbsPitchDeg, bool& bOutUseAbsolute, uint8* OutAbsCode)
{
	if (!RecvSock)
	{
		return false;
	}
	uint8 Buf[RecvBufSize];
	int32 Read = 0;
	TSharedRef<FInternetAddr> From = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();
	bool bGot = false;
	// 包长判据用 >= 20：基础包正好 20 字节；>= 29 时带绝对角扩展（偏移 20/24 两个 float + 偏移 28 一个 flag）。
	while (RecvSock->RecvFrom(Buf, sizeof(Buf), Read, *From) && Read >= 20)
	{
		uint32 Magic = 0;
		FMemory::Memcpy(&Magic, Buf, 4);
		if (Magic != MagicCmd)
		{
			continue;
		}
		FMemory::Memcpy(&OutDYawDeg, Buf + 4, 4);
		FMemory::Memcpy(&OutDPitchDeg, Buf + 8, 4);
		bOutFire = Buf[12] != 0;
		bOutHasTarget = Buf[13] != 0;
		if (Read >= 29)
		{
			FMemory::Memcpy(&OutAbsYawDeg, Buf + 20, 4);
			FMemory::Memcpy(&OutAbsPitchDeg, Buf + 24, 4);
			bOutUseAbsolute = Buf[28] != 0;
			if (OutAbsCode) { *OutAbsCode = Buf[28]; }
		}
		else
		{
			bOutUseAbsolute = false;
			if (OutAbsCode) { *OutAbsCode = 0; }
		}
		// ---- 检测结果搭车 ----
		// 偏移 29: uint8 nDets；偏移 30: uint32 frameId（检测所在的那一帧）；
		// 偏移 34 起：nDets×4 个点，每点 int16 x + int16 y。
		// 为什么并进指令包：41004 那条独立通道被"每次 Play 泄漏一个 socket"搞死过 ✗
		// （Windows 同端口多 socket 只有一个能收 ⇒ 回传被旧 socket 收走 ⇒ 绿色角点永远画不出来）。
		// 指令通道 41003 是实测**一直可靠**的（cmds 稳定增长 ✓），所以检测就走它，不再单开端口 ✓。
		if (Read >= 34)
		{
			const uint8 ND = Buf[29];
			if (ND <= 16 && Read >= 34 + (int32)ND * 16)
			{
				uint32 Fid = 0;
				FMemory::Memcpy(&Fid, Buf + 30, 4);
				Detections.Reset();
				for (int32 i = 0; i < (int32)ND; ++i)
				{
					for (int32 p = 0; p < 4; ++p)
					{
						int16 X = 0, Y = 0;
						FMemory::Memcpy(&X, Buf + 34 + (i * 4 + p) * 4, 2);
						FMemory::Memcpy(&Y, Buf + 34 + (i * 4 + p) * 4 + 2, 2);
						Detections.Add(FVector2D((float)X, (float)Y));
					}
				}
				DetCount = (int32)ND;
				DetFrameId = (int32)Fid;
				LastDetRecvTime = FPlatformTime::Seconds();
			}
		}
		++CmdsRecv;
		bGot = true;
	}
	// 低频日志：把"有没有收到回传"变成可观测事实（否则只能靠猜 ✗）
	if (DetCount > 0 && NowDetLogSeconds < LastDetRecvTime)
	{
		NowDetLogSeconds = LastDetRecvTime + 2.0;
		UE_LOG(LogTemp, Display, TEXT("WLAIMDET|recv %d plate(s) via the command packet"), DetCount);
	}
	// ★ 失效判定（用户要求：目标出视野**立刻**消失 ✓）：
	//   主判据用**帧号**（我相机又抓了 2 帧、却没收到属于新帧的检测 ⇒ 目标就是不在视野里了 ✓
	//   ≈66 ms 内消失，比原来的 0.25 秒超时干脆得多 ✓）；时间超时只作为兜底 ✓。
	const bool bFrameStale = (DetCount > 0) && (DetFrameId >= 0) && (LastFrameId - DetFrameId > 2);
	const bool bTimeStale = (DetCount > 0) && (LastDetRecvTime > 0.0) &&
		((FPlatformTime::Seconds() - LastDetRecvTime) > 0.15);
	if (bFrameStale || bTimeStale)
	{
		Detections.Reset();
		DetCount = 0;
		DetFrameId = -1;
	}
	return bGot;
}
