'use strict';

// Physical hosts and optional software services have separate configuration.
roleNames.nos='NOS · 导航主机';
roleNames.gos='GOS · 用户开发主机';
roleNames.sdk='SDK · 执行环境（可选）';
roleNames.train='外部训练工作站';

function hostTargetCard(role){
 const t=connections.targets[role];
 const defaults={nos:'/opt/robot/scripts/setup_ros2.sh',gos:'/opt/ros/foxy/setup.bash\n/opt/robot/scripts/setup_ros2.sh',sdk:'/opt/ros/foxy/setup.bash\n/opt/robot/scripts/setup_ros2.sh\n/home/'+t.user+'/M20_sdk_deploy/install/setup.bash',train:''};
 const hints={
  nos:'建图、定位、规划和地图文件保留在 NOS。平台使用此通道管理 drmap 与相关日志。',
  gos:'用户自定义节点优先部署在 GOS。此代理提供主机状态和只读 ROS2 数据。电脑上的平台后端和 YOLO 仍在电脑运行。',
  sdk:'这是软件执行环境，不代表第四台机器人主机。厂商真机示例在授权的 AOS 上运行；选择 GOS 前需核对 SDK 依赖、DDS 与控制交接。',
  train:'NVIDIA 工作站是机器人之外的训练资源，工程与 Python 环境独立配置。'
 };
 const common=`<label class="check"><input id="target-${role}-enabled" type="checkbox" ${t.enabled?'checked':''}><span>启用此通道，与其他主机同时保持连接</span></label><div class="form-grid">${field('主机 IP / 域名',role+'-host',t.host)}${field('SSH 用户',role+'-user',t.user)}${field('SSH 端口',role+'-ssh-port',t.ssh_port,'number')}${field('Python 绝对路径',role+'-python',t.python)}</div>`;
 let extra='';
 if(role==='nos'||role==='gos')extra=`<label class="check"><input id="${role}-via-aos" type="checkbox" ${t.via_aos?'checked':''}><span>经 AOS 中转连接（无需临时路由）</span></label><p class="help">文档常用地址：${role==='nos'?'10.21.31.106':'10.21.31.104'}，以本机配置为准。</p>`;
 if(role==='train')extra=field('rl_training 工程路径',role+'-train-root',t.train_root);
 if(role==='sdk')extra=`<label class="check"><input id="sdk-follows-aos" type="checkbox" ${(t.follows_aos??!t.host)?'checked':''}><span>SDK 与 AOS 同机，地址和账号自动跟随</span></label><p class="help">换网只需修改 AOS 地址。SDK 在其他电脑运行时取消此选项。</p>`+field('厂商 SDK 工作区',role+'-workspace',t.sdk_workspace||'/home/'+t.user+'/M20_sdk_deploy')+`<label class="check"><input id="sdk-admin" type="checkbox" ${t.sdk_admin?'checked':''}><span>按厂商示例以管理员运行 SDK 服务（首次部署需明确确认）</span></label>`;
 const environment=`${field('ROS / DDS domain ID',role+'-domain',t.domain_id??0,'number')}<div class="field"><label for="${role}-setup">运行环境脚本 / 每行一个绝对路径</label><textarea id="${role}-setup" rows="3">${esc(t.ros_setup.join('\n')||defaults[role])}</textarea></div><details class="advanced"><summary>高级连接参数</summary><div class="form-grid section-gap">${field('本地端口',role+'-local-port',t.local_port,'number')}${field('服务端口',role+'-remote-port',t.remote_port,'number')}</div></details>`;
 const controls=`<div class="connection-state" id="target-${role}-state">${esc(t.state.message)}</div><div class="button-row"><button type="button" class="btn primary small" data-setup-role="${role}">${t.deployed?'重新部署':'首次配置与部署'}</button><button type="button" class="btn small" data-connection-role="${role}" data-connection-operation="connect">连接</button><button type="button" class="btn small" data-connection-role="${role}" data-connection-operation="disconnect">断开</button></div><div class="button-row section-gap">${['start','restart','stop','logs'].map((op,i)=>`<button type="button" class="btn small" data-service-role="${role}" data-service-operation="${op}">${['启动服务','重启服务','停止服务','查看日志'][i]}</button>`).join('')}${role==='gos'?btn('查询 GOS 主机','gos-status','small'):''}</div>`;
 return card(roleNames[role],`<div class="card-pad">${common}${extra}${environment}${controls}<p class="help">${hints[role]}</p><p class="help">首次部署配置专用连接公钥与平台服务。密码仅用于当次部署，不保存；连接与运动控制分别启用。</p></div>`,'link',`<span class="pill ${t.state.status==='ready'?'':'gray'}" id="target-${role}-pill">${connectionNames[t.state.status]||t.state.status}</span>`);
}

connectionCenter=function(){
 if(!connections)return;
 const c=connections;
 const aos=card('AOS · 运动主机',`<div class="card-pad"><p class="help">原厂运控与 basic_server 位于 AOS。普通遥控和导航指令走协议通道，无需普通 AOS SSH 登录。</p>${field('机器人 AOS IP','hub-aos-ip',c.aos_ip)}${field('机器人 UDP 端口','hub-aos-port',c.aos_port,'number')}<details><summary>AOS 中转账号</summary>${field('AOS SSH 用户','hub-aos-ssh-user',c.aos_ssh_user||'user')}${field('AOS SSH 端口','hub-aos-ssh-port',c.aos_ssh_port||22,'number')}</details><div class="connection-state" id="aos-channel-state"></div>${btn('连接已启用的机器人通道','hub-connect','primary wide','link')}<div class="divider"></div>${kv('控制协议','basic_server / UDP')}${kv('底层运控','AOS 原厂控制器')}<p class="help">三台主机可同时通信；遥控、导航和底层 SDK 仍遵守控制权互斥。</p></div>`,'motion','<span class="pill gray">UDP · 30000</span>');
 const summary=`<div class="host-summary section-gap">${[['aos','AOS',c.aos_ip,'UDP 控制'],['nos','NOS',c.targets.nos.host||'地址待配置','地图 / SSH'],['gos','GOS',c.targets.gos.host||'地址待配置','用户开发 / SSH']].map(([role,title,address,purpose])=>`<div><strong>${title}</strong><span>${esc(address)}</span><small>${purpose}</small><b id="machine-${role}-state">—</b></div>`).join('')}</div>`;
 html('connection-content',`<form id="connections-profile-form"><section class="card card-pad"><strong>三主机协同连接</strong><p class="help">同时访问 AOS 的控制协议、NOS 的地图服务和 GOS 的用户开发环境。每条通道独立显示状态。</p><label class="check"><input id="hub-auto-connect" type="checkbox" ${c.auto_connect?'checked':''}><span>下次打开平台自动连接 AOS，并维护已启用的 NOS / GOS 通道；运动控制需要重新开启。</span></label><div class="button-row section-gap">${btn('保存全部连接配置','save-connections','primary')}${btn('同时连接已启用通道','hub-connect','','link')}${btn('使用演示环境','hub-demo')}</div></section>${summary}<div class="connection-grid host-grid section-gap">${aos}${hostTargetCard('nos')}${hostTargetCard('gos')}</div><section class="card card-pad section-gap">${select('实时 ROS2 数据接入主机','hub-sensor-source',[['nos','NOS · 保留当前数据接入'],['gos','GOS · 用户侧数据接入']])}<p class="help">GOS 接收雷达点云时需核对 multicast-relay 和 DDS domain。切换数据来源不会迁移 NOS 的建图导航服务。当前平台后端与 YOLO 运行在电脑。</p></section><details class="optional-services section-gap"><summary>可选软件执行环境与外部训练工作站</summary><p class="help">SDK 是功能服务，执行端按厂商流程与已验证环境配置；训练工作站是外部资源。</p><div class="connection-grid service-grid section-gap">${hostTargetCard('sdk')}${hostTargetCard('train')}</div></details></form>`);
 $('#hub-sensor-source').value=c.sensor_source||'nos';
 paintAOSChannel();paintSDKAddressFollow();
};

readConnections=function(){
 const targets={};
 for(const role of ['nos','gos','sdk','train']){
  targets[role]={via_aos:!!$('#'+role+'-via-aos')?.checked,host:$('#'+role+'-host').value.trim(),user:$('#'+role+'-user').value.trim(),enabled:$('#target-'+role+'-enabled').checked,
   ssh_port:+$('#'+role+'-ssh-port').value,python:$('#'+role+'-python').value.trim(),local_port:+$('#'+role+'-local-port').value,remote_port:+$('#'+role+'-remote-port').value,
   ros_setup:$('#'+role+'-setup').value.split('\n').map(v=>v.trim()).filter(Boolean),train_root:$('#'+role+'-train-root')?.value||'',sdk_workspace:$('#'+role+'-workspace')?.value||'',
   sdk_admin:role==='sdk'&&$('#sdk-admin').checked,domain_id:+$('#'+role+'-domain').value};
 }
 targets.sdk.follows_aos=!!$('#sdk-follows-aos')?.checked;
 return {aos_ssh_user:$('#hub-aos-ssh-user').value.trim(),aos_ssh_port:+$('#hub-aos-ssh-port').value,aos_ip:$('#hub-aos-ip').value.trim(),aos_port:+$('#hub-aos-port').value,auto_connect:$('#hub-auto-connect').checked,sensor_source:$('#hub-sensor-source').value,targets};
};

function paintSDKAddressFollow(){
 const checked=$('#sdk-follows-aos')?.checked;
 for(const [sdk,aos] of [['sdk-host','hub-aos-ip'],['sdk-user','hub-aos-ssh-user'],['sdk-ssh-port','hub-aos-ssh-port']]){
  const input=$('#'+sdk),source=$('#'+aos);if(!input||!source)continue;
  input.disabled=!!checked;if(checked)input.value=source.value;
 }
}
document.addEventListener('change',e=>{if(e.target.id==='sdk-follows-aos')paintSDKAddressFollow();});
document.addEventListener('input',e=>{if(['quick-aos-ip','hub-aos-ip','hub-aos-ssh-user','hub-aos-ssh-port'].includes(e.target.id))paintSDKAddressFollow();});

function paintAOSChannel(){text('aos-channel-state',S?.mode==='demo'?'演示环境 / 未连接实机':S?.online?'收到有效 UDP 状态':'等待有效 UDP 状态');text('machine-aos-state',S?.mode==='demo'?'演示':S?.online?'协议在线':'等待回包');for(const role of ['nos','gos'])text('machine-'+role+'-state',connectionNames[connections?.targets[role]?.state.status]||'未连接');}
const previousConnectionsLoad=loadConnections;
loadConnections=async function(paint=false){await previousConnectionsLoad(paint);paintAOSChannel();};
ui['gos-status']=async()=>{const r=await api('/api/gos/status');modal('GOS 主机状态',`<pre class="pre">${esc(JSON.stringify(r,null,2))}</pre>`);};
