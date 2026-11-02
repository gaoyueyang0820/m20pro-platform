"""M20 Pro business modes and versioned gait names from vendor documentation."""
MODES=[dict(action='mode_normal',value=0,name='常规 / 手动',description='摇杆直接控制'),
       dict(action='mode_aux',value=2,name='辅助 / 避障',description='手动操控，雷达辅助避障'),
       dict(action='mode_nav',value=1,name='自主导航',description='执行地图中的目标和路线')]
GAITS=[dict(action='gait_basic',value=0x1001,name='基础',group='标准运动',legacy=False),
       dict(action='gait_standard_stair',value=0x1003,name='楼梯',group='标准运动',legacy=False),
       dict(action='gait_flat',value=0x3002,name='平地',group='敏捷运动',legacy=False),
       dict(action='gait_stair',value=0x3003,name='楼梯',group='敏捷运动',legacy=False),
       dict(action='gait_platform',value=0x1002,name='高台',group='版本兼容',legacy=True)]
def catalog():
    return dict(ok=True,modes=MODES,gaits=GAITS,
        note='3 种使用模式；4 项现行运动章节步态，另保留综合协议中的高台步态。手柄固件版本可能有命名差异。',
        sources=['259675892 使用模式管理','259675903 basic_server 运动控制','259675872 软件开发指南 Gait 表'])
