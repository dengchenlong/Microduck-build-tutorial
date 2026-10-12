# 保留 OpenRB-150 作 USB-串口透传（不换 UART 直连）

主控到舵机总线保留 OpenRB-150 当 USB 串口透传（FanGe 原方案，~¥150）。Pi Zero 2W UART 直连 + SN74LVC1G126 半双工缓冲（Xgoduck 开源方案）可省这 ¥150，但要自己打样小板 + 改系统串口配置，多 1–2 天活且引入新硬件风险。

决策原则：硬件不动，把软件风险集中在协议改写这一个点上。UART 直连留给 v2。

## Consequences

- 驱动层串口探测路径为 OpenRB-150 实际路径（原为 `/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_*`）
- v2 升级时需要打样半双工小板
