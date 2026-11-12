'use strict';

// Connection and map workflows extend the existing controls without changing the
// supplied UDP/video/YOLO implementation.
const hubPageOrder=['overview','motion','sdk','joints','navigation','maps','tasks','training','sensors','vision','diagnostics','settings'];
const savedPageDefinitions={...pages,sdk:['SDK 控制','SDK WORKSPACE','连接 SDK，切换策略状态或控制关节。']};
savedPageDefinitions.maps=['地图与点云','MAP WORKSPACE','浏览三维点云、管理地图包，并把地图同步到导航工作区。'];
savedPageDefinitions.settings=['连接中心','CONNECTION CENTER','连接机器人，查看各主机状态并管理设备。'];
for(const key of Object.keys(pages))delete pages[key];
for(const key of hubPageOrder)pages[key]=savedPageDefinitions[key];
if(pages[location.hash.slice(1)])page=location.hash.slice(1);
paths.sdk=paths.code;
let connections=null,mapLibrary={maps:[]},fileLibrary=[],cloudViewer=null,cloudData=null,selectedRemote=null,cloudSource=null;
let cloudViewGeneration=0;
let connectionLoading=false,libraryLoading=false,setupRole=null,setupGeneration=0,sdkInfo=null,sdkPolicy=null,hubMode=null;
const oldMapsPage=mapsPage,baseRender=render,baseUpdate=update;
const sizeLabel=bytes=>bytes==null?'—':bytes>=1024**3?(bytes/1024**3).toFixed(2)+' GiB':bytes>=1024**2?(bytes/1024**2).toFixed(1)+' MiB':(bytes/1024).toFixed(1)+' KiB';
const roleNames={nos:'NOS · 地图与建图',sdk:'SDK · 模式与关节',train:'训练工作站'};
const connectionNames={idle:'尚未连接',connecting:'正在连接',ready:'服务在线',waiting_service:'服务无响应',unreachable:'网络未接通',stopped:'已停止',needs_setup:'需要首次配置'};

function operationPanel(){return '<div class="operation-box" id="hub-operation" hidden><div><strong id="hub-op-label">后台任务</strong><span id="hub-op-percent">0%</span></div><div class="progress"><span id="hub-op-bar" style="width:0%"></span></div><p id="hub-op-message">准备中</p></div>';}
function setOperation(label,progress,message,status){
 $('#hub-operation')?.removeAttribute('hidden');
 text('hub-op-label',label);
 text('hub-op-percent',status==='failed'?'失败':status==='completed'?'已完成':Math.round(progress*100)+'%');
 text('hub-op-message',message);
 const bar=$('#hub-op-bar');
 if(bar){bar.parentElement.hidden=status==='failed';bar.style.width=(progress*100)+'%';}
}
async function waitJob(start,label,isCurrent=()=>true,quiet=false){
 if(!start.job_id)return isCurrent()?start:null;
 const waitingSince=Date.now();
 for(;;){
  if(!isCurrent())return null;
  const response=await api('/api/operations?id='+encodeURIComponent(start.job_id),undefined,quiet);
  if(!isCurrent())return null;
  const job=response.jobs;
  let message=job.error||job.message;
  if(job.status==='running'&&job.kind?.startsWith('deploy-'))message+=` · 已等待 ${Math.floor((Date.now()-waitingSince)/1000)} 秒`;
  setOperation(label,job.progress,message,job.status);
  if(job.status==='failed')throw Error(job.error||'后台操作失败');
  if(job.status==='completed')return job.result;
  await new Promise(resolve=>setTimeout(resolve,500));
 }
}
async function job(path,body,label,isCurrent){return waitJob(await api(path,body,!!isCurrent),label,isCurrent,!!isCurrent);}

settings=function(){return heading(btn('设备维护','maintenance-modal','','settings')+btn('导出工作区','export','','download'))+operationPanel()+`<div id="connection-content">${empty('正在读取连接配置','连接配置保存在本机。','link')}</div>`;};
function connectionCenter(){
 if(!connections)return;
 const c=connections;
 const demo=S?.mode==='demo',online=!!S?.online,ip=S?.ip||c.aos_ip||'机器人地址待配置';
 const cards=Object.entries(c.targets).map(([role,t])=>card(roleNames[role],`<div class="card-pad"><label class="check"><input id="target-${role}-enabled" type="checkbox" ${t.enabled?'checked':''}><span>启用此目标，平台在实机连接时维护后台通道</span></label><div class="form-grid">${field('主机 IP / 域名',role+'-host',t.host)}${field('SSH 用户',role+'-user',t.user)}${field('SSH 端口',role+'-ssh-port',t.ssh_port,'number')}${field('Python 绝对路径',role+'-python',t.python)}</div>${role==='train'?field('rl_training 工程路径',role+'-train-root',t.train_root):''}${role==='sdk'?field('厂商 SDK 工作区',role+'-workspace',t.sdk_workspace||'/home/'+t.user+'/M20_sdk_deploy')+`<label class="check"><input id="sdk-admin" type="checkbox" ${t.sdk_admin?'checked':''}><span>按厂商示例以管理员运行 SDK 服务（首次部署需明确确认）</span></label>`:''}${field('ROS / DDS domain ID',role+'-domain',t.domain_id??0,'number')}<div class="field"><label for="${role}-setup">运行环境脚本 / 每行一个绝对路径</label><textarea id="${role}-setup" rows="3">${esc(t.ros_setup.join('\n')||((role==='nos')?'/opt/robot/scripts/setup_ros2.sh':role==='sdk'?'/opt/ros/foxy/setup.bash\n/opt/robot/scripts/setup_ros2.sh\n/home/'+t.user+'/M20_sdk_deploy/install/setup.bash':''))}</textarea></div><details open class="advanced"><summary>高级连接参数</summary><div class="form-grid section-gap">${field('本地端口',role+'-local-port',t.local_port,'number')}${field('服务端口',role+'-remote-port',t.remote_port,'number')}</div></details><div class="connection-state" id="target-${role}-state">${esc(t.state.message)}</div><div class="button-row"><button type="button" class="btn primary small" data-setup-role="${role}">${t.deployed?'重新部署':'首次配置与部署'}</button><button type="button" class="btn small" data-connection-role="${role}" data-connection-operation="connect">连接</button><button type="button" class="btn small" data-connection-role="${role}" data-connection-operation="disconnect">断开</button></div><div class="button-row section-gap"><button type="button" class="btn small" data-service-role="${role}" data-service-operation="start">启动服务</button><button type="button" class="btn small" data-service-role="${role}" data-service-operation="restart">重启服务</button><button type="button" class="btn small" data-service-role="${role}" data-service-operation="stop">停止服务</button><button type="button" class="btn small" data-service-role="${role}" data-service-operation="logs">查看日志</button></div><p class="help">${role==='sdk'?'SDK 连接服务可以常驻；策略控制和关节输出均需在 SDK 页面手动开启。':'首次部署会配置专用连接公钥和开机服务。密码仅用于这次连接，不保存。'}</p></div>`,'link',`<span class="pill ${t.state.status==='ready'?'':'gray'}" id="target-${role}-pill">${connectionNames[t.state.status]||t.state.status}</span>`)).join('');
 const hostStrip=Object.entries(c.targets).map(([role,t])=>`<div><strong>${role.toUpperCase()}<small>${role==='nos'?'建图、定位与导航':role==='sdk'?'SDK 执行环境':'外部训练工作站'}</small></strong><span>${esc(t.host||'地址待配置')}</span><span>${connectionNames[t.state.status]||'尚未连接'}</span></div>`).join('');
 html('connection-content',`<section class="card card-pad quick-connect"><h2>连接机器人 <span class="pill ${demo?'amber':online?'green':'gray'}">${demo?'离线演示':online?'已连接':'等待回包'}</span></h2><div class="connection-state" id="connection-summary">${esc(demo?'离线演示 · 未连接实机，遥测与地图均为模拟数据。':online?'已连接 '+ip+' · UDP 状态正常；运动控制需在驾驶页重新开启。':'等待回包 · '+ip+'：请检查电脑与机器人是否在同一网络。')}</div><div class="quick-connect-form">${field('机器人 AOS IP','hub-aos-ip',c.aos_ip)}${field('机器人 UDP 端口','hub-aos-port',c.aos_port,'number')}${btn('连接机器人','hub-connect','primary','link')}${btn('离线体验','hub-demo')}${demo?'':btn('断开','hub-disconnect','small ghost','link')}</div><p class="help">连接后平台自动维护已启用的主机通道；运动控制与关节控制需要在各操作页面重新开启。首次安装只使用一次设备密码，密码不保存。</p></section><section class="card section-gap"><div class="card-header"><h2 class="card-title">主机状态</h2><span class="help">${demo?'演示环境不连接实机':online?'实机链路在线':'等待实机回包'}</span></div><div class="card-pad"><div class="host-summary" style="margin:0">${hostStrip}</div></div></section><details open class="connection-advanced card section-gap" id="connection-advanced"><summary>高级管理：账号、服务与网络设置</summary><p class="help card-pad">机器人地址与连接方式见页面上方。这里保存主机账号、运行环境脚本与端口，修改后下次连接生效。</p><form id="connections-profile-form"><div class="card-pad"><label class="check"><input id="hub-auto-connect" type="checkbox" ${c.auto_connect?'checked':''}><span>下次打开平台自动连接 AOS，并维护已启用的 NOS / SDK / 训练通道；运动控制需要重新开启。</span></label><div class="button-row section-gap">${btn('保存全部连接配置','save-connections','')}</div></div><div class="connection-grid host-grid section-gap">${cards}</div></form></details>`);
 for(const primary of $$('#connections-profile-form .btn.primary'))primary.classList.remove('primary');
 paintConnectionSummary();
}
function paintConnectionSummary(){
 if(!connections)return;
 const summary=$('#connection-summary');if(!summary)return;
 const demo=S?.mode==='demo',online=!!S?.online,ip=S?.ip||connections.aos_ip||'机器人地址待配置';
 summary.textContent=demo?'离线演示 · 未连接实机，遥测与地图均为模拟数据。':online?('已连接 '+ip+' · UDP 状态正常；运动控制需在驾驶页重新开启。'):('等待回包 · '+ip+'：请检查电脑与机器人是否在同一网络。');
}
ui['hub-disconnect']=async()=>{
 const roles=Object.entries(connections?.targets||{}).filter(([,target])=>target.enabled).map(([role])=>role);
 if(!roles.length){toast('没有启用的后台通道需要断开');return;}
 for(const role of roles)await api('/api/connections/target',{role,action:'stop'});
 await loadConnections(true);toast('已断开后台通道：'+roles.map(role=>roleNames[role]||role).join(' / '));
};
function readConnections(){
 const targets={};
 for(const role of ['nos','sdk','train'])targets[role]={host:$('#'+role+'-host').value.trim(),user:$('#'+role+'-user').value.trim(),enabled:$('#target-'+role+'-enabled').checked,
 ssh_port:+$('#'+role+'-ssh-port').value,python:$('#'+role+'-python').value.trim(),local_port:+$('#'+role+'-local-port').value,remote_port:+$('#'+role+'-remote-port').value,
 ros_setup:$('#'+role+'-setup').value.split('\n').map(v=>v.trim()).filter(Boolean),train_root:$('#'+role+'-train-root')?.value||'',sdk_workspace:$('#'+role+'-workspace')?.value||'',sdk_admin:role==='sdk'&&$('#sdk-admin').checked,domain_id:+$('#'+role+'-domain').value};
 return {aos_ip:$('#hub-aos-ip').value.trim(),aos_port:+$('#hub-aos-port').value,auto_connect:$('#hub-auto-connect').checked,targets};
}
async function loadConnections(paint=false){
 if(connectionLoading)return;connectionLoading=true;
 try{connections=(await api('/api/connections',undefined,true)).connections;if(paint)connectionCenter();else for(const [role,t] of Object.entries(connections.targets)){text('target-'+role+'-state',t.state.message);text('target-'+role+'-pill',connectionNames[t.state.status]||t.state.status);}}
 finally{connectionLoading=false;}
}
ui['save-connections']=async()=>{if(!$('#connections-profile-form'))return;await api('/api/connections/save',readConnections());await loadConnections();toast('配置已保存在本机，下次启动会使用这些设置');};
ui['connect-modal']=()=>navigate('settings');
ui['hub-connect']=async()=>{if($('#connections-profile-form'))await api('/api/connections/save',readConnections());await api('/api/connections/connect',{mode:'live'});session=null;await fetchState();toast('正在连接机器人');};
ui['hub-demo']=async()=>{await api('/api/connections/connect',{mode:'demo'});session=null;await fetchState();toast('已切换演示环境');};
let setupInstallIntent=null,setupDeployBusy=false;
function setupConnectionScope(role){const t=connections?.targets?.[role];return JSON.stringify([S?.server_instance,S?.connection_generation,connections?.aos_ip,t?.host,t?.ssh_port]);}
function setupDeviceCurrent(token){return page===token.page&&setupConnectionScope(token.role)===token.scope;}
function setupInstallCurrent(token){return setupInstallIntent===token&&setupGeneration===token.generation&&setupDeviceCurrent(token)&&$('#modal')?.open!==false&&(token.dialog===null||typeof mapDialogGeneration==='undefined'||mapDialogGeneration===token.dialog);}
async function setupWizard(role,{reinstall=false}={}){
 if($('#connections-profile-form')){await api('/api/connections/save',readConnections());await loadConnections();}
 const t=connections.targets[role];if(!t?.host)throw Error('尚未找到设备地址，请先自动查找机器人');
 if(t.deployed&&!reinstall){await api('/api/connections/target',{role,action:'start'});await loadConnections();toast('正在连接 '+roleNames[role]);return;}
 setupRole=role;setupGeneration++;
 modal((reinstall?'更新连接服务 · ':'首次连接 · ')+roleNames[role],`<p>设备：${esc(t.host)}。安装连接服务后，日常可直接连接。</p>${field('设备账号','setup-user',t.user||'user')}${field('设备密码','setup-password','','password')}${field('管理员密码（需要时填写）','setup-admin-password','','password')}${role==='sdk'?'<details open><summary>关节校准文件（可选）</summary><a href="/calibration.example.json" download>下载校准模板</a><input type="file" id="setup-calibration" accept=".json"></details>':''}<p class="help">将安装平台连接服务并配置自动启动。密码只用于本次安装，不会保存。</p>${btn(reinstall?'更新并连接':'安装并连接','setup-deploy','primary wide')}<div id="setup-message" class="help"></div>`);
 setupInstallIntent={role,generation:setupGeneration,scope:setupConnectionScope(role),page,dialog:typeof mapDialogGeneration==='number'?mapDialogGeneration:null,reinstall};
 for(const id of ['setup-password','setup-admin-password'])$('#'+id).autocomplete='new-password';
}
ui['setup-deploy']=async()=>{
 const token=setupInstallIntent;if(!token||!setupInstallCurrent(token))throw Error('连接设置已变化，请重新打开首次连接');if(setupDeployBusy)return;
 const password=$('#setup-password'),adminPassword=$('#setup-admin-password');
 const req={role:token.role,password:password.value,admin_password:adminPassword.value,confirmed:true};
 const button=$('[data-ui="setup-deploy"]');setupDeployBusy=true;if(button)button.disabled=true;
 text('setup-message','');
 try{
  const user=$('#setup-user').value.trim();if(!user)throw Error('请输入设备账号');
  if($('#connections-profile-form')){const input=$('#'+token.role+'-user');if(input)input.value=user;if(token.role==='sdk'&&connections.targets.sdk.follows_aos&&$('#hub-aos-ssh-user')){$('#hub-aos-ssh-user').value=user;if(typeof paintSDKAddressFollow==='function')paintSDKAddressFollow();}await api('/api/connections/save',readConnections());}
  if(!setupInstallCurrent(token))return;
  if($('#setup-calibration')?.files[0])req.calibration=JSON.parse(await $('#setup-calibration').files[0].text());
  if(!setupInstallCurrent(token))return;
  const result=await job('/api/connections/deploy',req,token.reinstall?'更新连接服务':'安装并连接设备',()=>setupDeviceCurrent(token));
  if(!result||!setupDeviceCurrent(token))return;
  const dialogOpen=setupInstallCurrent(token);
  password.value='';adminPassword.value='';
  if(dialogOpen)$('#modal').close();
  if(setupInstallIntent===token)setupInstallIntent=null;
  await loadConnections(dialogOpen);toast(result.message||'连接服务已就绪');
 }catch(error){
  if(setupInstallCurrent(token))text('setup-message',error.message);
  throw error;
 }finally{setupDeployBusy=false;if(button?.isConnected!==false)button.disabled=false;}
};

mapsPage=function(){
 const legacy=oldMapsPage();const index=legacy.indexOf('<div class="nav-layout">');
 return heading(btn('本机 drmap 参数','map-vendor-help','','code')+btn('刷新地图库','map-library-refresh','','refresh')+btn('上传地图文件','open-map-upload','primary','plus'))+operationPanel()+
 `<div class="map-workbench"><section class="card map-library"><div class="card-header"><h2 class="card-title">${icon('maps')}机器人地图库</h2><span class="pill" id="map-library-source">${S.mode==='demo'?'演示 NOS':'NOS'}</span></div><div id="remote-map-list"></div><div class="library-footer"><span id="map-disk">—</span></div></section><section class="card"><div class="card-header"><h2 class="card-title">${icon('sensors')}三维点云地图</h2><span id="cloud-source" class="pill gray">尚未加载</span></div><div class="cloud-stage"><div id="cloud-view" class="cloud-canvas"></div><div class="cloud-overlay"><strong id="cloud-title">选择地图或上传 PCD</strong><small id="cloud-subtitle">支持 ASCII / binary / binary_compressed</small></div><div class="cloud-toolbox">${btn('复位视角','cloud-fit','small')}${btn('浅色背景','cloud-background','small')}</div><div class="cloud-empty" id="cloud-empty">${icon('maps')}<strong>点云地图工作区</strong><span>查看机器人地图，或预览电脑中的 PCD / ZIP 地图包</span></div><div class="cloud-bottom"><span>拖动旋转 · 右键平移 · 滚轮缩放</span><span>坐标单位 / m</span></div></div><div class="cloud-controls"><label>点尺寸 <input type="range" id="cloud-size" min="0.015" max="0.2" step="0.005" value="0.065"></label><label>最低高度 <input type="number" id="cloud-zmin" step="0.1" value="-1"></label><label>最高高度 <input type="number" id="cloud-zmax" step="0.1" value="10"></label><label><input id="cloud-grid" type="checkbox" checked> 网格</label><label><input id="cloud-mono" type="checkbox"> 单色</label></div><div class="live-cloud-controls"><div class="field"><label for="live-cloud-topic">实时 ROS2 点云 / 保留原话题坐标系</label><select id="live-cloud-topic"><option value="/SLAM_ALIGNED_POINTS">建图实时扫描 / SLAM_ALIGNED_POINTS</option><option value="/SLAM_ACCUMULATED_POINTS_MAP">建图累积点云 / SLAM_ACCUMULATED_POINTS_MAP</option><option value="/ALIGNED_POINTS">配准点云 / ALIGNED_POINTS</option><option value="/FULL_CLOUD_MAP">全局地图 / FULL_CLOUD_MAP</option><option value="/LOC_BODY_POINTS">机身点云 / LOC_BODY_POINTS</option><option value="/LIDAR/POINTS">原始雷达 / LIDAR/POINTS</option><option value="/LIDAR/POINTS2">后雷达 / LIDAR/POINTS2</option></select></div>${btn('查看实时点云','cloud-live','small')}${btn('暂停接收','cloud-live-stop','small')}</div><div class="cloud-metrics" id="cloud-metrics">预览按需抽取点云，导出文件保留完整地图。</div></section></div>`+
 `<div class="equal-split section-gap">${card('电脑文件与地图包',`<form class="card-pad" id="hub-upload-form"><div class="field"><label for="hub-map-file">选择 PCD 点云或 ZIP 地图包 / 支持分块上传</label><input type="file" id="hub-map-file" accept=".pcd,.zip" required></div><div class="button-row"><button class="btn primary" type="submit">上传到平台</button>${btn('刷新文件','file-library-refresh')}</div><p class="help">文件留在当前电脑。导入机器人需单独确认。</p><div id="upload-status" class="help"></div></form><div id="local-file-list"></div>`,'file')}${card('选中地图操作',`<div class="card-pad"><p class="help" id="selected-map-info">先在地图库选择一份地图。</p>${select('栅格编号','remote-grid-id',[[0,'栅格 0']])}<div class="form-grid">${btn('地图内容与栅格映射','map-details','','file')}${btn('加载三维点云','preview-selected-map','','sensors')}${btn('查看二维栅格','sync-selected-map','','maps')}${btn('导出完整地图包','export-selected-map','','download')}${btn('切换为活动地图','apply-selected-map','primary')}</div><div class="divider"></div>${field('新建地图名称','hub-map-name','m20-map')}<div class="form-grid">${btn('开始建图','hub-mapping-start')}${btn('停止并保存','hub-mapping-stop')}</div><p class="help">地图切换会重启定位。导入新地图包时不会自动覆盖或切换原活动地图。</p></div>`,'settings')}</div>`+
 `<section class="card section-gap"><div class="card-header"><h2 class="card-title">${icon('maps')}建图采集与后处理</h2>${btn('刷新建图状态','map-build-status','small')}</div><div class="card-pad" id="mapping-progress"><p class="help">读取建图进程与厂商日志…</p></div></section>`+
 `<details open class="legacy-map-tools section-gap" id="grid-tools"><summary>${icon('maps')} 二维栅格与定位工具</summary><div class="section-gap">${index>=0?legacy.slice(index):legacy}</div></details>`;
};
async function loadMapLibrary(){
 if(libraryLoading)return;libraryLoading=true;
 try{mapLibrary=await api('/api/maps/library',undefined,true);if(!selectedRemote||!mapLibrary.maps.some(m=>m.name===selectedRemote))selectedRemote=mapLibrary.active||mapLibrary.maps[0]?.name;paintMapLibrary();}
 catch(e){const list=$('#remote-map-list');if(list)delete list._listMarkup;html('remote-map-list',empty('地图服务未连接','在连接中心完成 NOS 首次配置后，这里会自动列出机器人地图。','link'));text('map-disk',e.message);}
 finally{libraryLoading=false;}
}
function paintMapLibrary(){
 const listMarkup=mapLibrary.maps.map(m=>`<button class="map-library-item ${m.name===selectedRemote?'selected':''}" data-remote-map="${esc(m.name)}"><span class="map-thumb">${icon('maps')}</span><span><strong>${esc(m.name)}</strong><small>${m.grids.length} 份栅格 · ${sizeLabel(m.cloud_bytes)}</small></span>${m.active?'<span class="dot"></span>':''}</button>`).join('')||empty('还没有地图','可通过平台开始建图，或上传并导入地图包。','maps');const list=$('#remote-map-list');if(list&&list._listMarkup!==listMarkup){list.innerHTML=listMarkup;list._listMarkup=listMarkup;}
 text('map-disk',mapLibrary.disk?'存储可用 '+sizeLabel(mapLibrary.disk.free):'—');
 const selected=mapLibrary.maps.find(m=>m.name===selectedRemote);
 text('selected-map-info',selected?selected.name+(selected.active?' · 当前活动地图':' · 未激活'):'请选择地图');
 const gridMarkup=selected?.grids.map(i=>`<option value="${i}">栅格 ${i}</option>`).join('')||'<option value="0">栅格 0</option>';const grid=$('#remote-grid-id');if(grid&&(grid._gridMarkup!==gridMarkup||grid._mapName!==selectedRemote)){const previous=grid._mapName===selectedRemote?grid.value:'0';grid.innerHTML=gridMarkup;grid._gridMarkup=gridMarkup;grid._mapName=selectedRemote;if([...grid.options].some(o=>o.value===previous))grid.value=previous;}
}
async function loadFiles(){fileLibrary=(await api('/api/files')).files;paintFiles();}
function paintFiles(){html('local-file-list',fileLibrary.map(f=>`<div class="file-library-row"><div><strong>${esc(f.name)}</strong><small>${f.kind==='pcd'?'PCD 点云':'ZIP 地图包'} · ${sizeLabel(f.size)} · SHA256 ${esc(f.sha256.slice(0,10))}…</small></div><div class="button-row"><button type="button" class="btn small" data-file-preview="${f.id}">三维预览</button>${f.kind==='map-package'?`<button type="button" class="btn small" data-file-inspect="${f.id}">内容</button><button type="button" class="btn small primary" data-file-import="${f.id}">导入机器人</button>`:''}<a class="btn small" href="/api/files/download/${f.id}" download>下载</a></div></div>`).join('')||empty('电脑文件区为空','上传后可预览、检查、下载，或导入机器人。','file'));}
async function uploadFile(file){
 const kind=file.name.toLowerCase().endsWith('.pcd')?'pcd':file.name.toLowerCase().endsWith('.zip')?'map-package':null;
 if(!kind)throw Error('请选择 PCD 点云或 ZIP 地图包');
 const stamp=[file.name,file.size,file.lastModified].join('|'),saved=JSON.parse(localStorage.getItem('m20-upload-resume')||'null');
 const req={name:file.name,size:file.size,kind};if(saved?.stamp===stamp)req.resume_id=saved.id;
 const begin=await api('/api/files/start',req);const meta=begin.file;
 localStorage.setItem('m20-upload-resume',JSON.stringify({stamp,id:meta.id}));
 for(let offset=meta.offset;offset<file.size;){
  const blob=file.slice(offset,offset+begin.chunk_size);const encoded=await readFileBase64(blob);
  const result=await api('/api/files/chunk',{id:meta.id,offset,data:encoded});offset=result.offset;
  text('upload-status',`${Math.round(offset/file.size*100)}% · ${sizeLabel(offset)} / ${sizeLabel(file.size)}`);
  setOperation('上传文件',offset/file.size,'分块传输中，网络中断后可重新选择同一文件续传');
 }
 const result=await job('/api/files/finish',{id:meta.id},'校验上传文件');
 localStorage.removeItem('m20-upload-resume');text('upload-status','上传并校验完成');await loadFiles();toast('文件已保存在当前电脑');
 return result.file;
}
function setupCloudViewer(){
 const el=$('#cloud-view');if(!el)return;
 if(!window.createCloudViewer){setTimeout(()=>{if(el.isConnected)setupCloudViewer();},100);return;}
 try{cloudViewer=window.createCloudViewer(el);if(cloudData)showCloud(cloudData);}
 catch(e){text('cloud-empty','点云显示需要 WebGL，请启用浏览器硬件加速。');}
}
function showCloud(data,fitView=true){
 cloudData=data;cloudViewer?.load(data,fitView);$('#cloud-empty')?.setAttribute('hidden','');
 text('cloud-title',data.name);text('cloud-source',data.source==='ros2'?('实时 ROS2 · '+(data.fresh?'已接收':'接收过期')):data.source==='demo'?'演示 / 合成点云':data.source==='nos'?'机器人地图':'电脑文件');
 text('cloud-subtitle',`${data.encoding.toUpperCase()} · XYZ / m${data.frame_id?' · FRAME '+data.frame_id:''}`);
 text('cloud-metrics',`原始 ${data.total_points.toLocaleString()} 点 · 预览 ${data.shown_points.toLocaleString()} 点${data.sampled?' / 已采样':''} · 样本范围 X ${fmt(data.bounds[0][0])}…${fmt(data.bounds[1][0])} m / Y ${fmt(data.bounds[0][1])}…${fmt(data.bounds[1][1])} m`);
 if(fitView&&$('#cloud-zmin'))$('#cloud-zmin').value=fmt(data.bounds[0][2]-.1,2);
 if(fitView&&$('#cloud-zmax'))$('#cloud-zmax').value=fmt(data.bounds[1][2]+.1,2);
}
async function previewCloud(kind,key){
 if(page!=='maps')return;
 liveCloudTopic=null;const generation=++cloudViewGeneration,mode=S.mode,ip=S.ip;
 const current=()=>generation===cloudViewGeneration&&page==='maps'&&S.mode===mode&&S.ip===ip;
 try{
  const result=await job('/api/cloud/preview',{kind,key},'读取并采样点云',current);if(!current()||!result)return;
  const data=await api('/api/cloud/data/'+result.cloud_id,undefined,true);if(!current())return;
  cloudSource={kind,key};showCloud(data);
 }catch(e){if(current())throw e;}
}
ui['map-library-refresh']=loadMapLibrary;ui['file-library-refresh']=loadFiles;
ui['open-map-upload']=()=>$('#hub-map-file').click();
ui['preview-selected-map']=()=>{if(!selectedRemote)throw Error('请先选择地图');return previewCloud('map',selectedRemote);};
ui['sync-selected-map']=async()=>{if(!selectedRemote)throw Error('请先选择地图');const result=await api('/api/maps/library/sync',{name:selectedRemote,grid_id:+$('#remote-grid-id').value});mapId=result.map.id;await fetchState();render();$('#grid-tools').open=true;$('#grid-tools').scrollIntoView({behavior:'smooth'});toast('二维栅格已同步，可编辑航点与初始定位');};
ui['export-selected-map']=async()=>{if(!selectedRemote)throw Error('请先选择地图');const result=await job('/api/maps/package/export',{name:selectedRemote},'导出完整地图包');await loadFiles();toast(result.format==='drmap'?'厂商 drmap 地图包已下载到电脑文件区':'完整目录备份已下载；包格式标为目录备份，实机兼容性仍需核对');};
ui['apply-selected-map']=()=>{if(!selectedRemote)throw Error('请先选择地图');modal('切换活动地图',`<p>将使用 ${esc(selectedRemote)} 并重启定位服务。完成后请等待地图加载，再设置并核对机器人位置。</p>`,[{label:'确认切换',ui:'hub-confirm-apply'}]);};
ui['hub-confirm-apply']=async()=>{$('#modal').close();await job('/api/maps/library/apply',{name:selectedRemote},'切换活动地图');await loadMapLibrary();toast('地图切换请求已完成，请核对定位状态');};
ui['hub-mapping-start']=()=>job('/api/maps/library/build',{action:'start',name:$('#hub-map-name').value},'开始建图').then(()=>toast(S.mode==='demo'?'演示环境不采集真实雷达数据':'建图已请求启动，请查看雷达与服务状态'));
ui['hub-mapping-stop']=()=>job('/api/maps/library/build',{action:'stop'},'停止并保存地图').then(async()=>{await loadMapLibrary();toast('建图停止请求已完成，请在地图库核对结果');});
ui['cloud-fit']=()=>cloudViewer?.reset();
ui['cloud-background']=()=>{cloudViewer?.style({light:$('#cloud-view')?.dataset.light!=='true'});const el=$('#cloud-view');if(el)el.dataset.light=el.dataset.light==='true'?'false':'true';};

function sdkPage(){
 return heading(btn('刷新状态','sdk-refresh','small','refresh')+btn('结束 SDK 控制','sdk-exit','danger','stop'))+operationPanel()+`<div class="sdk-steps" aria-label="SDK 操作步骤"><span><b>1</b>手柄开启 SDK 模式</span><span><b>2</b>连接服务</span><span><b>3</b>站起</span><span><b>4</b>策略行走</span><span><b>5</b>返回日常驾驶</span></div><section class="sdk-welcome card card-pad"><div><span class="eyebrow">SDK CONTROL</span><h2>SDK 控制</h2><p>先用手柄开启 SDK 模式，再连接。首次使用请安装运行环境。</p><div class="button-row">${btn('已开启 SDK，连接','sdk-connect','primary','play')}${btn('安装 SDK 环境','sdk-setup','','code')}</div></div><div id="sdk-environment" class="sdk-environment">正在检查工程</div></section><div class="sdk-console-grid">${card('SDK 控制',`<div class="card-pad"><div id="sdk-checks"></div><div class="sdk-state-actions">${btn('站起','sdk-stand','primary')}${btn('策略行走','sdk-rl','primary')}${btn('趴下','sdk-lie')}${btn('阻尼','sdk-damping')}</div><div class="sdk-motion"><div class="dpad"><button data-sdk-velocity="0,0,0.2" aria-label="SDK 左转">↶</button><button data-sdk-velocity="0.1,0,0" aria-label="SDK 前进">↑</button><button data-sdk-velocity="0,0,-0.2" aria-label="SDK 右转">↷</button><button data-sdk-velocity="0,0.1,0" aria-label="SDK 左移">←</button><button class="center" data-ui="sdk-velocity-stop">停止</button><button data-sdk-velocity="0,-0.1,0" aria-label="SDK 右移">→</button><span></span><button data-sdk-velocity="-0.1,0,0" aria-label="SDK 后退">↓</button><span></span></div><p class="help">站起后进入策略行走，再按住方向键移动；松开停止。</p></div>${kv('策略状态','—','sdk-policy-state')}${kv('控制来源','—','sdk-policy-source')}${kv('反馈','—','sdk-policy-fresh')}</div>`,'motion')}<div class="stack">${card('返回日常驾驶',`<div class="card-pad"><p>结束平台 SDK 控制，在手柄退出 SDK 后，点击下方返回。</p>${btn('手柄已退出，返回驾驶','sdk-return','','motion')}</div>`,'motion')}${card('关节与单腿',`<div class="card-pad"><p>进入关节页面可控制单腿、单关节和姿态。</p>${btn('打开关节控制','goto-joints','','joints')}<p class="help">独立关节动作需要吊架和关节校准。</p></div>`,'joints')}<details open class="card card-pad"><summary>工程配置与日志</summary><div class="button-row section-gap">${btn('SDK 日志','sdk-service-logs','small')}${btn('连接配置','goto-connections','small')}${btn('停止策略进程','sdk-policy-stop','small')}${btn('停止关节进程','sdk-direct-stop','small')}</div><div id="sdk-mode-observation"></div></details></div></div>`;
}
async function loadSDK(){
 try{
  sdkInfo=await api('/api/sdk/status',undefined,true);
  try{const response=await api('/api/sdk/policy/status',undefined,true);sdkPolicy=response.policy;paintSDK(response.error);}
  catch(e){sdkPolicy=null;paintSDK(e.message);}
 }catch(e){text('sdk-environment','无法读取 SDK 状态：'+e.message);}
}
function paintSDK(error){
 if(!sdkInfo)return;
 const q=sdkInfo,env=q.environment||{};
 html('sdk-checks',kv('连接服务',q.agent_ready?'已连接':'未连接')+kv('手柄 SDK',q.state==='sdk'?'已记录开启':'等待手柄开启'));
 html('sdk-environment',`<span class="pill ${env.ready?'':'amber'}">${q.demo?'离线演示':env.ready?'工程就绪':'需要准备'}</span><p>${esc(q.demo?'演示 SDK · 不控制电机':env.message||'连接服务后可读取工程状态')}</p>${env.workspace?'<small>'+esc(env.workspace)+'</small>':''}`);
 text('sdk-policy-state',({idle:'空闲 / 阻尼',stand:'站立',rl_control:'策略行走',lie_down:'趴下'})[sdkPolicy?.current_state]||sdkPolicy?.current_state||'等待启动');text('sdk-policy-source',sdkPolicy?.source||'等待反馈');text('sdk-policy-fresh',error||((sdkPolicy?.fresh&&sdkPolicy?.telemetry_fresh)?'已接收':'等待状态回报'));
 html('sdk-mode-observation',kv('模式记录来源',q.source==='operator_confirmed'?'手柄操作确认':q.source==='demo'?'演示':'未确认')+kv('硬件状态码',S.basic.MotionState??'未知'));
}
ui['sdk-connect']=async()=>{const r=await job('/api/sdk/connect',{},'连接 SDK');toast(r.message||'已请求启动，等待状态回报');await loadSDK();};
ui['sdk-setup']=async()=>{const r=await job('/api/sdk/setup',{},'安装 SDK 环境');toast(r.message||'环境准备完成');await loadSDK();};
ui['sdk-return']=async()=>{if(!['normal','awaiting_handset_exit'].includes(sdkInfo?.state))await api('/api/sdk/action',{action:'exit'});await api('/api/sdk/action',{action:'confirm_exit',confirmed:true});navigate('motion');};
ui['goto-connections']=()=>navigate('settings');ui['goto-joints']=()=>navigate('joints');ui['sdk-refresh']=loadSDK;
ui['sdk-exit']=async()=>{sdkVelocityStop();const r=await api('/api/sdk/action',{action:'exit'});toast(r.message);await loadSDK();};
ui['sdk-service-logs']=async()=>{const r=await api('/api/sdk/logs');modal('SDK 运行日志',`<pre class="pre">${esc(r.output)}</pre>`);};
ui['sdk-policy-stop']=async()=>{const r=await api('/api/sdk/policy/stop',{});toast(r.message||'已请求停止策略程序');await loadSDK();};
ui['sdk-direct-stop']=async()=>{await api('/api/sdk/policy/direct-stop',{});toast('已请求停止单关节调试服务');await loadSDK();};
for(const [key,state] of Object.entries({'sdk-stand':'stand','sdk-rl':'rl_control','sdk-lie':'lie_down','sdk-damping':'damping'}))ui[key]=async()=>{const r=await api('/api/sdk/policy/state',{state});toast(r.message||'SDK 状态请求已发送');await loadSDK();};
let sdkMotionTimer=null,sdkVector=[0,0,0];
function sdkVelocityStop(){if(sdkMotionTimer){clearInterval(sdkMotionTimer);sdkMotionTimer=null;api('/api/sdk/policy/velocity',{forward:0,side:0,yaw:0},true).catch(()=>{});}}
ui['sdk-velocity-stop']=()=>{sdkVelocityStop();api('/api/sdk/policy/velocity',{forward:0,side:0,yaw:0},true).catch(()=>{});};
function sdkVelocityStart(vector){sdkVector=vector;const send=()=>api('/api/sdk/policy/velocity',{forward:sdkVector[0],side:sdkVector[1],yaw:sdkVector[2]},true).catch(e=>{sdkVelocityStop();toast(e.message,true);});if(!sdkMotionTimer){send();sdkMotionTimer=setInterval(send,100);}}
async function serviceOperation(role,action){
 const r=await job('/api/connections/service',{role,action},roleNames[role]+' · '+action);
 if(action==='logs'||action==='status')modal(roleNames[role]+' 日志',`<pre class="pre">${esc(r.output||r.message||'无输出')}</pre>`);
 else toast('服务操作已完成，请查看连接状态');await loadConnections();
}

render=function(){
 sdkVelocityStop();cloudViewGeneration++;liveCloudTopic=null;if(hubMode!==S.mode){cloudData=null;cloudSource=null;selectedRemote=null;hubMode=S.mode;}if(cloudViewer){cloudViewer.dispose();cloudViewer=null;}
 baseRender();
 if(page==='settings')loadConnections(true).catch(()=>{});
 if(page==='maps'){setupCloudViewer();loadBuildStatus().catch(()=>{});loadMapLibrary().then(()=>{if(!cloudData&&selectedRemote)previewCloud('map',selectedRemote).catch(()=>{});});loadFiles().catch(()=>{});}
 if(page==='sdk')loadSDK();if(page==='sensors')loadROSSensors().catch(()=>{});
};
update=function(){baseUpdate();if(page==='maps'&&cloudViewer&&cloudSource?.kind==='map'){const active=mapLibrary.maps.find(m=>m.name===cloudSource.key)?.active;cloudViewer.pose(S.pose,active&&S.pose.Location===0);}};

document.addEventListener('click',async event=>{
 const button=event.target.closest('button');if(!button)return;
 try{
  if(button.dataset.setupRole)return await setupWizard(button.dataset.setupRole,{reinstall:button.dataset.setupUpdate==='true'});
  if(button.dataset.connectionRole){const role=button.dataset.connectionRole;await api('/api/connections/target',{role,action:button.dataset.connectionOperation==='disconnect'?'stop':'start'});await loadConnections();return;}
  if(button.dataset.serviceRole){await serviceOperation(button.dataset.serviceRole,button.dataset.serviceOperation);return;}
  if(button.dataset.remoteMap){selectedRemote=button.dataset.remoteMap;paintMapLibrary();await previewCloud('map',selectedRemote);return;}
  if(button.dataset.filePreview){await previewCloud('file',button.dataset.filePreview);return;}
  if(button.dataset.fileInspect){const r=await api('/api/files/inspect',{id:button.dataset.fileInspect});modal('地图包内容',`<p>${r.package.files} 个文件，展开后 ${sizeLabel(r.package.unpacked_size)}</p><pre class="pre">${esc(r.package.contents.map(f=>f.name+' · '+sizeLabel(f.size)).join('\n'))}</pre>`);return;}
  if(button.dataset.fileImport){
   const id=button.dataset.fileImport,file=fileLibrary.find(f=>f.id===id);const inspection=await api('/api/files/inspect',{id});
   modal('导入地图包到'+(S.mode==='demo'?'演示机器人':'机器人'),`<p>将 ${esc(file.name)} 导入为一份新地图，现有地图和活动地图保持不变。</p>${field(S.mode==='demo'?'新的演示地图名称':'包内原地图名称','package-import-name',inspection.package.map_name||file.name.replace(/\.zip$/i,'').replace(/[^\w\u4e00-\u9fff.-]/g,'_'))}`,[{label:'确认导入',ui:'confirm-package-import'}]);
   ui['confirm-package-import']=async()=>{const name=$('#package-import-name').value;$('#modal').close();await job('/api/maps/package/import',{id,name},'导入地图包');selectedRemote=name;await loadMapLibrary();toast('地图包已导入，尚未切换活动地图');};
  }
 }catch(e){if(!$('.toast.error'))toast(e.message,true);}
});
document.addEventListener('submit',async event=>{
 if(event.target.id==='hub-upload-form'){
  event.preventDefault();const file=$('#hub-map-file').files[0];if(!file)return;
  const submit=event.target.querySelector('button[type="submit"]');submit.disabled=true;
  try{const value=await uploadFile(file);if(value.kind==='pcd')await previewCloud('file',value.id);}catch(e){text('upload-status','上传中断：'+e.message+'。重新选择同一文件可继续。');}finally{submit.disabled=false;}
 }
 if(event.target.id==='connections-profile-form'){event.preventDefault();try{await ui['save-connections']();}catch(e){}}
});
document.addEventListener('input',event=>{
 if(!cloudViewer)return;
 const id=event.target.id;
 if(id==='cloud-size')cloudViewer.style({size:+event.target.value});
 if(id==='cloud-zmin')cloudViewer.style({zmin:+event.target.value});
 if(id==='cloud-zmax')cloudViewer.style({zmax:+event.target.value});
 if(id==='cloud-grid')cloudViewer.style({grid:event.target.checked});
 if(id==='cloud-mono')cloudViewer.style({mono:event.target.checked});
});
document.addEventListener('pointerdown',event=>{const target=event.target.closest('[data-sdk-velocity]');if(!target)return;event.preventDefault();target.setPointerCapture(event.pointerId);sdkVelocityStart(target.dataset.sdkVelocity.split(',').map(Number));});
for(const name of ['pointerup','pointercancel'])document.addEventListener(name,sdkVelocityStop);
window.addEventListener('blur',sdkVelocityStop);document.addEventListener('visibilitychange',()=>{if(document.hidden)sdkVelocityStop();});
setInterval(()=>{if(document.hidden)return;if(page==='settings')loadConnections().catch(()=>{});if(page==='sdk')loadSDK().catch(()=>{});},2000);


ui['map-details']=async()=>{
 if(!selectedRemote)throw Error('请先选择地图');
 const r=await api('/api/maps/details',{name:selectedRemote});
 modal('地图文件与栅格映射',`<h3>${esc(r.name)}</h3><p>栅格编号按 occ_grid_编号.pgm 读取；原厂 ID 对应关系保留在以下文件中。</p><pre class="pre">${esc(r.grid_mapping||'此地图未提供 occ_grid_id_map.toml')}</pre><pre class="pre">${esc(r.files.map(f=>f.name+' · '+sizeLabel(f.size)).join('\n'))}</pre>`);
};
ui['map-vendor-help']=async()=>{const r=await job('/api/maps/help',{},'读取目标版本 drmap 参数');modal('目标设备的 drmap 帮助',`<pre class="pre">${esc(r.output)}</pre>`);};
let lastBuildEnd=null,buildLoading=false;
async function loadBuildStatus(){
 if(buildLoading)return;buildLoading=true;
 try{
  const r=await api('/api/maps/build/status',undefined,true);
  html('mapping-progress',r.demo?'<p class="help">演示环境不采集真实雷达数据。实机连接后显示采集进程、停止后处理状态及原厂日志，不虚构百分比。</p>':['mapping','stop_mapping'].map((kind,i)=>{const q=r[kind]||{};return `<div class="mapping-record"><strong>${i?'停止与后处理':'建图采集'} · ${esc(({running:'运行中',command_finished:'命令已结束',failed:'失败',unknown:'结果待核对',none:'暂无记录'})[q.state]||q.state)}</strong><p class="help">${esc(q.message||'')}</p>${q.output?`<details open><summary>查看厂商日志</summary><pre class="pre">${esc(q.output)}</pre></details>`:''}</div>`;}).join(''));
  const end=r.stop_mapping?.ended;
  if(end&&lastBuildEnd!==end){lastBuildEnd=end;await loadMapLibrary();}
 }catch(e){text('mapping-progress','建图状态暂不可读：'+e.message);}
 finally{buildLoading=false;}
}
ui['map-build-status']=loadBuildStatus;
nosAction=async function(action){text('nos-output','正在请求 NOS…');try{const r=action?await job('/api/nos/action',{action,name:$('#nos-map-name').value},'NOS 地图操作'):await api('/api/nos/status');text('nos-output',JSON.stringify(r,null,2));if(action)await loadMapLibrary();}catch(e){text('nos-output',e.message);}};
setInterval(()=>{if(page==='maps'&&!document.hidden)loadBuildStatus().catch(()=>{});},3000);
document.addEventListener('keydown',event=>{if(event.code==='Space'&&!event.target.matches('input,textarea,select'))sdkVelocityStop();});
window.addEventListener('beforeunload',()=>{if(sdkMotionTimer)navigator.sendBeacon('/api/sdk/policy/velocity',new Blob([JSON.stringify({forward:0,side:0,yaw:0})],{type:'application/json'}));});


let liveCloudTopic=null,liveCloudBusy=false,liveCloudReceived=false,rosBusy=false;
ui['cloud-live']=async()=>{
 if(S.mode==='demo')throw Error('实时通道需要机器人 ROS2 数据；演示地图可直接在地图库预览');
 cloudViewGeneration++;liveCloudTopic=$('#live-cloud-topic').value;liveCloudReceived=false;await loadLiveCloud();
};
ui['cloud-live-stop']=()=>{cloudViewGeneration++;liveCloudTopic=null;text('cloud-source','实时接收已暂停 / 保留最后一帧');cloudViewer?.pose({},false);};
async function loadLiveCloud(){
 if(liveCloudBusy||!liveCloudTopic||page!=='maps')return;liveCloudBusy=true;
 const topic=liveCloudTopic,generation=cloudViewGeneration;
 try{
  const data=await api('/api/sensors/cloud',{topic},true);
  if(generation!==cloudViewGeneration||topic!==liveCloudTopic||page!=='maps')return;
  showCloud(data,!liveCloudReceived);liveCloudReceived=true;cloudSource={kind:'live',key:topic};
  const odom=data.odom;cloudViewer?.pose(odom||{},!!(data.fresh&&odom));
  text('cloud-metrics',`${topic} · 坐标系 ${data.frame_id||'未提供'} · 原始 ${data.total_points.toLocaleString()} 点 / 预览 ${data.shown_points.toLocaleString()} 点 · 接收于 ${data.age.toFixed(1)} 秒前${data.fresh?'':' / 数据已过期'}`);
 }catch(e){if(generation===cloudViewGeneration&&topic===liveCloudTopic&&page==='maps'){text('cloud-source','等待实时 ROS2 数据');text('cloud-metrics',e.message);cloudViewer?.pose({},false);}}
 finally{liveCloudBusy=false;}
}
const oldSensorsPage=sensors;
sensors=function(){return oldSensorsPage()+`<div class="section-gap">${card('ROS2 原始数据通道',`<div class="card-pad"><p class="help">原始 IMU、定位里程计和点云由连接中心选择的 GOS / NOS 只读订阅。保留话题坐标系；接收时间和数据源分别显示。</p><div class="button-row">${btn('刷新 ROS2 数据','ros-refresh')}${btn('前往点云工作区','goto-pointcloud')}</div><div id="ros-sensor-details" class="section-gap">正在读取 ROS2 数据…</div></div>`,'sensors')}</div>`;};
ui['goto-pointcloud']=()=>navigate('maps');ui['ros-refresh']=()=>loadROSSensors();
async function loadROSSensors(){
 if(rosBusy)return;rosBusy=true;
 try{
  const r=await api('/api/sensors/status',undefined,true),now=Date.now()/1000;
  if(r.demo){text('ros-sensor-details','演示环境不伪造原始 ROS2 数据。连接已配置的数据主机后在这里查看实测数据。');return;}
  const vector=v=>v?.map(n=>fmt(n,3)).join(' / ')||'等待数据';
  const age=q=>q?Math.max(0,now-q.received_at).toFixed(1)+' s':'未收到';
  html('ros-sensor-details',kv('ROS2 接入主机',esc((r.host_role||'nos').toUpperCase()))+kv('IMU / 坐标系',esc(r.imu?.frame_id||'等待数据'))+kv('角速度 / rad/s',vector(r.imu?.angular_velocity))+kv('加速度 / m/s²',vector(r.imu?.linear_acceleration))+kv('IMU 接收间隔',age(r.imu))+kv('ODOM / 坐标系',esc(r.odom?.frame_id||'等待数据'))+kv('定位 X / Y / Z',r.odom?[r.odom.PosX,r.odom.PosY,r.odom.PosZ].map(v=>fmt(v,3)).join(' / '):'等待数据')+kv('定位接收间隔',age(r.odom))+`<div class="table-wrap section-gap"><table><thead><tr><th>话题</th><th>坐标系</th><th>接收间隔</th></tr></thead><tbody>${r.topics.map(t=>`<tr><td>${esc(t.topic)}</td><td>${esc(t.frame_id||'—')}</td><td>${t.available?fmt(t.age,1)+' s':esc(t.error||'未收到')}</td></tr>`).join('')}</tbody></table></div>`);
 }catch(e){text('ros-sensor-details','ROS2 通道未就绪：'+e.message);}
 finally{rosBusy=false;}
}
setInterval(()=>{if(document.hidden)return;if(page==='maps')loadLiveCloud();if(page==='sensors')loadROSSensors();},1200);

