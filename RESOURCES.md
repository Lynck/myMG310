# CH9141K 双车通信资源

## Knowledge

- [CH9141 官方数据手册 - WCH](https://www.wch.cn/downloads/CH9141DS1_PDF.html)
  芯片和 AT 指令的第一来源。遇到不同批次固件返回值不一致时优先核对这里。
- [BleUartApp 官方工具 - WCH](https://www.wch.cn/downloads/BleUartApp_ZIP.html)
  用于电脑或手机连接从机并做 BLE 串口测试。
- [BleComWin 官方工具 - WCH](https://www.wch.cn/downloads/BleComWin_ZIP.html)
  用于在 Windows 上建立 CH9141 无线虚拟串口。
- `C:\Users\Lenovo\Desktop\生产力\电子信息\TI\资料\蓝牙模块\CH9141K 使用说明-张栩豪.pdf`
  三页实操说明，重点覆盖 `AT...`、角色设置、扫描连接和 `CONADD` 自动重连。
- `C:\Users\Lenovo\Desktop\生产力\电子信息\TI\资料\蓝牙模块\CH9141K蓝牙模块使用说明-信工.pdf`
  十一页模块说明，包含模块原理图、接口排列、完整主从连接表和状态查询指令。
- `C:\Users\Lenovo\Desktop\生产力\电子信息\TI\资料\C题-小车跟随行驶系统 (1).pdf`
  比赛规则原文；用于核对“跟随车仅上电开关”和“只能车间通信”的约束。

## Gaps

- 两份实操说明对 `AT+CONADD` 是否同时返回 `LINK OK` 的描述不一致，最终应以本模块固件的 `AT+BLESTA?`、`AT+CCADD?` 和掉电重连实测为准。
