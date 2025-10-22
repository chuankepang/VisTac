## 1. Rokae 机械臂连接
### 临时修改 IP 地址

临时修改 IP 地址适用于测试，系统重启后会恢复原配置。

### 步骤

1. 检查当前 IP 配置：

    已确认 eth0 的 IP 为 192.168.55.55/24，需要更改为 192.168.0.100/24。


2. 删除当前 IP 地址：

    如果 eth0 已有 IP 地址，先删除：
    > sudo ip addr del 192.168.55.55/24 dev eth0



3. 添加新 IP 地址：

    为 eth0 设置新 IP 地址 192.168.0.100：
    > sudo ip addr add 192.168.0.100/24 dev eth0



4. 验证 IP 修改：

    检查 eth0 的新配置：
    > ip addr show eth0

    输出如下

    > eth0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc mq state UP group default qlen 1000
    
    > inet 192.168.0.100/24 scope global eth0



5. 测试与机械臂的连接：

    确认能否 ping 通机械臂：
    > ping 192.168.0.160

    如果 ping 成功，说明网络配置正确，