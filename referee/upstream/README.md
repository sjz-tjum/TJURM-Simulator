# 上游参考（Arena-1）

来源：<https://github.com/RM-Arena/Arena-1>
版权归上游，按其 LICENSE 使用；本目录只放**规则数据与文档**，Rust 源码请直接看上游：

- simulation/src/lib.rs（78KB）核心数据结构 + Engine
- simulation/src/official.rs（112KB）官方规则
- simulation/src/season2027.rs（95KB）2027 赛季
- simulation/src/penalties.rs（11KB）判罚
- simulation/src/physics.rs（103KB）
- simulation/src/*_tests.rs 规则测试（移植对照用例来源）

本目录文件：
- ules.json：全部数值参数（370KB）
- RULES_2027_IMPLEMENTATION.md：实现说明
- RULES_IMPLEMENTATION_AUDIT.md：实现审计（偏差清单）
