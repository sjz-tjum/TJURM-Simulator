# 操作端

## tools/
| 脚本 | 用途 |
|---|---|
| `bias_window.py/.bat` | 起 peer + 键盘热调 yaw/pitch 偏置（可见控制台） |
| `run_aim_test.bat` | 一键跑自瞄测试 |
| `place_vehicle_in_level.py` | 在关卡里摆车/摆靶 |
| `verify_vehicle_in_level.py` | 校验车/靶位置 |
| `make_pvp_levels.py` | 生成 PVP 关卡 |
| `gt_calib.py` | 真值标定 |
| `led_final.py / led_emissive.py` | 灯条自发光材质（亮度） |

## hud/
- `WLBoostHUD.*`：屏幕显示检测绿框/概率、PnP 与真值对照、偏置状态
