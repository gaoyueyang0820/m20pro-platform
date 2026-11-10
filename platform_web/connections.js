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
  nos:'负责建图、定位、导航和地图存储。',
  gos:'用于扩展程序与传感器数据接入。',
  sdk:'SDK 运行主机，默认跟随 AOS 地址与账号。',
  train:'配置用于训练的 NVIDIA 工作站。'
 };
 const common=`<label class="check"><input id="target-${role}-enabled" type="checkbox" ${t.enabled?'checked':''}><span>启用此通道，与其他主机同时保持连接</span></label><div class="form-grid">${field('主机 IP / 域名',role+'-host',t.host)}${field('SSH 用户',role+'-user',t.user)}${field('SSH 端口',role+'-ssh-port',t.ssh_port,'number')}${field('Python 绝对路径',role+'-python',t.python)}</div>`;
 let extra='';
 if(role==='nos'||role==='gos')extra=`<label class="check"><input id="${role}-via-aos" type="checkbox" ${t.via_aos?'checked':''}><span>经 AOS 中转连接（无需临时路由）</span></label><p class="help">默认地址：${role==='nos'?'10.21.31.106':'10.21.31.104'}，以本机配置为准。</p>`;
 if(role==='train')extra=field('rl_training 工程路径',role+'-train-root',t.train_root);
 if(role==='gos')extra=field('机内 AOS 地址（GOS 导航）','gos-onboard-aos',t.gos_aos_ip||'');
 if(role==='sdk')extra=`<label class="check"><input id="sdk-follows-aos" type="checkbox" ${(t.follows_aos??!t.host)?'checked':''}><span>SDK 与 AOS 同机，地址和账号自动跟随</span></label><p class="help">换网只需修改 AOS 地址。SDK 在其他电脑运行时取消此选项。</p>`+field('厂商 SDK 工作区',role+'-workspace',t.sdk_workspace||'/home/'+t.user+'/M20_sdk_deploy')+`<label class="check"><input id="sdk-admin" type="checkbox" ${t.sdk_admin?'checked':''}><span>使用管理员权限运行 SDK 服务</span></label>`;
 const environment=`${field('ROS / DDS domain ID',role+'-domain',t.domain_id??0,'number')}<div class="field"><label for="${role}-setup">运行环境脚本 / 每行一个绝对路径</label><textarea id="${role}-setup" rows="3">${esc(t.ros_setup.join('\n')||defaults[role])}</textarea></div><details class="advanced"><summary>高级连接参数</summary><div class="form-grid section-gap">${field('本地端口',role+'-local-port',t.local_port,'number')}${field('服务端口',role+'-remote-port',t.remote_port,'number')}</div></details>`;
 const controls=`<div class="connection-state" id="target-${role}-state">${esc(t.state.message)}</div><div class="button-row"><button type="button" class="btn primary small" data-setup-role="${role}" ${t.deployed?'data-setup-update="true"':''}>${t.deployed?'更新服务':'首次安装'}</button><button type="button" class="btn small" data-connection-role="${role}" data-connection-operation="connect">连接</button><button type="button" class="btn small" data-connection-role="${role}" data-connection-operation="disconnect">断开</button></div><div class="button-row section-gap">${['start','restart','stop','logs'].map((op,i)=>`<button type="button" class="btn small" data-service-role="${role}" data-service-operation="${op}">${['启动服务','重启服务','停止服务','查看日志'][i]}</button>`).join('')}${role==='gos'?btn('查询 GOS 主机','gos-status','small'):''}<button type="button" class="btn small" data-pair-role="${role}">重新配对</button></div>`;
 return card(roleNames[role],`<div class="card-pad">${common}${extra}${environment}${controls}<p class="help">${hints[role]}</p><p class="help">首次安装后可直接连接。设备密码不会保存。</p></div>`,'link',`<span class="pill ${t.state.status==='ready'?'':'gray'}" id="target-${role}-pill">${connectionNames[t.state.status]||t.state.status}</span>`);
}

connectionCenter=function(){
 if(!connections)return;
 const c=connections;
 const aos=card('AOS · 运动主机',`<div class="card-pad"><p class="help">负责机器人运动控制。</p>${field('机器人 AOS IP','hub-aos-ip',c.aos_ip)}${field('机器人 UDP 端口','hub-aos-port',c.aos_port,'number')}<details><summary>AOS 中转账号</summary>${field('AOS SSH 用户','hub-aos-ssh-user',c.aos_ssh_user||'user')}${field('AOS SSH 端口','hub-aos-ssh-port',c.aos_ssh_port||22,'number')}</details><div class="connection-state" id="aos-channel-state"></div><button type="button" class="btn small" data-pair-role="aos">重新配对 AOS</button><div class="divider"></div>${kv('控制协议','basic_server / UDP')}${kv('底层运控','AOS 原厂控制器')}<p class="help">三个主机会同时连接，控制方式在各操作页面选择。</p></div>`,'motion','<span class="pill gray">UDP · 30000</span>');
 const summary=`<div class="host-summary section-gap">${[['aos','AOS',c.aos_ip,'UDP 控制'],['nos','NOS',c.targets.nos.host||'地址待配置','地图 / SSH'],['gos','GOS',c.targets.gos.host||'地址待配置','用户开发 / SSH']].map(([role,title,address,purpose])=>`<div><strong>${title}</strong><span>${esc(address)}</span><small>${purpose}</small><b id="machine-${role}-state">—</b></div>`).join('')}</div>`;
 html('connection-content',`<form id="connections-profile-form"><section class="card card-pad"><strong>三主机协同连接</strong><p class="help">设置需要连接的主机及服务。</p><label class="check"><input id="hub-auto-connect" type="checkbox" ${c.auto_connect?'checked':''}><span>下次打开平台自动连接 AOS，并维护已启用的 NOS / GOS 通道；运动控制需要重新开启。</span></label><div class="button-row section-gap">${btn('保存全部连接配置','save-connections','primary')}${btn('同时连接已启用通道','hub-connect','','link')}${btn('使用演示环境','hub-demo')}</div></section>${summary}<div class="connection-grid host-grid section-gap">${aos}${hostTargetCard('nos')}${hostTargetCard('gos')}</div><section class="card card-pad section-gap">${select('实时 ROS2 数据接入主机','hub-sensor-source',[['nos','NOS · 保留当前数据接入'],['gos','GOS · 用户侧数据接入']])}<p class="help">选择传感器数据来源。建图与导航继续由 NOS 提供。</p></section><details class="optional-services section-gap"><summary>可选软件执行环境与外部训练工作站</summary><p class="help">按需配置 SDK 运行主机和训练工作站。</p><div class="connection-grid service-grid section-gap">${hostTargetCard('sdk')}${hostTargetCard('train')}</div></details></form>`);
 $('#hub-sensor-source').value=c.sensor_source||'nos';
 paintAOSChannel();paintSDKAddressFollow();
};

readConnections=function(){
 const targets={};
 for(const role of ['nos','gos','sdk','train']){
  targets[role]={host:$('#'+role+'-host').value.trim(),user:$('#'+role+'-user').value.trim(),enabled:$('#target-'+role+'-enabled').checked,
   ssh_port:+$('#'+role+'-ssh-port').value,python:$('#'+role+'-python').value.trim(),local_port:+$('#'+role+'-local-port').value,remote_port:+$('#'+role+'-remote-port').value,
   ros_setup:$('#'+role+'-setup').value.split('\n').map(v=>v.trim()).filter(Boolean),train_root:$('#'+role+'-train-root')?.value||'',sdk_workspace:$('#'+role+'-workspace')?.value||'',
   sdk_admin:role==='sdk'&&$('#sdk-admin').checked,domain_id:+$('#'+role+'-domain').value};
  // The backend can switch internal hosts to the relay while this form is open.
  const relay=$('#'+role+'-via-aos');
  if(relay&&relay.checked!==relay.defaultChecked)targets[role].via_aos=relay.checked;
  if(role==='gos'&&$('#gos-onboard-aos'))targets[role].gos_aos_ip=$('#gos-onboard-aos').value.trim();
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
loadConnections=async function(paint=false){
 await previousConnectionsLoad(paint);
 for(const role of ['nos','gos']){
  const relay=$('#'+role+'-via-aos'),saved=connections?.targets[role]?.via_aos;
  if(relay&&typeof saved==='boolean'&&(relay.checked===relay.defaultChecked||relay.checked===saved)){
   relay.defaultChecked=saved;
   relay.checked=saved;
  }
 }
 paintAOSChannel();
};
ui['gos-status']=async()=>{const r=await api('/api/gos/status');modal('GOS 主机状态',`<pre class="pre">${esc(JSON.stringify(r,null,2))}</pre>`);};
let connectionDiscoveryBusy=false,connectionDiscoveryGeneration=0;
ui['discover-robot']=async()=>{
 if(connectionDiscoveryBusy)return;connectionDiscoveryBusy=true;const generation=++connectionDiscoveryGeneration,server=S?.server_instance;
 const button=$('[data-ui="discover-robot"]');if(button)button.disabled=true;
 try{
  const result=await job('/api/connections/discover',{},'查找已配对的机器人',()=>generation===connectionDiscoveryGeneration&&S?.server_instance===server);
  if(!result||generation!==connectionDiscoveryGeneration||S?.server_instance!==server)return;
  if(result.connections)connections=result.connections;else await loadConnections();
  if(page==='settings')connectionCenter();toast(result.message||(result.found?'已找到机器人，可以连接':'未找到机器人，请检查 Wi-Fi 和电源'));
 }finally{if(generation===connectionDiscoveryGeneration){connectionDiscoveryBusy=false;const current=$('[data-ui="discover-robot"]');if(current)current.disabled=false;}}
};
function repairDevicePairing(role){
 const scope=setupConnectionScope(role),address=role==='aos'?connections.aos_ip:connections.targets[role]?.host;
 if(!address)throw Error('请先设置设备地址');
 modal('重新配对设备',`<p>重新保存 ${esc(role.toUpperCase())}（${esc(address)}）的设备身份。仅在机器人重装或更换后使用，请确认地址对应你的设备。</p>`,[{label:'重新配对','ui':'confirm-device-pairing'}]);
 const dialog=typeof mapDialogGeneration==='number'?mapDialogGeneration:null;
 ui['confirm-device-pairing']=async()=>{
  if(scope!==setupConnectionScope(role)||dialog!==null&&typeof mapDialogGeneration!=='undefined'&&dialog!==mapDialogGeneration)throw Error('连接设置或窗口已变化，请重新选择设备');
  $('#modal').close();const result=await job('/api/connections/pair',{role,replace:true},'重新配对设备',()=>scope===setupConnectionScope(role));
  if(!result||scope!==setupConnectionScope(role))return;await loadConnections();toast(result.message||'设备已重新配对');
 };
}
document.addEventListener('click',async event=>{const button=event.target.closest('[data-pair-role]');if(!button)return;try{repairDevicePairing(button.dataset.pairRole);}catch(error){toast(error.message,true);}});
