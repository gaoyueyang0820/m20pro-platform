'use strict';

// Page copy for the primary work pages; module/tab navigation is rendered by app.js.
Object.assign(pages,{
 overview:['概览','OVERVIEW','设备连接、运行状态与当前任务的统一视图。'],
 motion:['手动驾驶','DRIVE','相机画面、底盘遥控与步态模式。'],
 maps:['建图与地图','MAPPING','建图采集、地图库与点云。'],
 navigation:['导航与巡检','NAVIGATION','单点导航任务与多航点巡检路线的规划与执行。']
});
overview=function(){
 const entry=(key,title,tip)=>`<button type="button" data-page="${key}"><span class="q-icon">${icon(key)}</span><span><strong>${title}</strong><small>${tip}</small></span><span class="arrow">›</span></button>`;
 const entries=[['motion','手动驾驶','相机、遥控与步态模式'],['navigation','导航与巡检','单点导航与多航点巡检'],['maps','建图与地图','建图采集、地图库与点云'],['autonomy','自主作业','自主探索与人员跟随'],['vision','视觉识别','相机预览、人员与 PPE 检测'],['diagnostics','故障与日志','错误码与原始回包排查']];
 const entryColumn=list=>`<div class="quick-entry">${list.map(([key,title,tip])=>entry(key,title,tip)).join('')}</div>`;
 return heading(btn('导出状态','export','','download')+btn('连接机器人','connect-modal','primary','link'))+
 `<div class="kpi-grid">
   <div class="card kpi"><div class="kpi-label">${icon('link')}设备连接</div><div class="kpi-value" id="kpi-online">在线<small>DEMO</small></div><div class="kpi-note"><span class="dot"></span><span id="kpi-ip">本地演示环境</span></div></div>
   <div class="card kpi"><div class="kpi-label">${icon('battery')}双电池电量</div><div class="kpi-value" id="kpi-battery">—</div><div class="kpi-note" id="kpi-battery-note">前侧 / 后侧电池</div><svg class="kpi-graph" viewBox="0 0 100 40"><path d="M1 31L15 24 28 27 40 17 53 20 66 13 81 15 99 6" fill="none" stroke="#70bfa5" stroke-width="1.5"/></svg></div>
   <div class="card kpi"><div class="kpi-label">${icon('joints')}关节状态</div><div class="kpi-value" id="kpi-joints">—</div><div class="kpi-note">16 个电机 · 四条腿</div></div>
   <div class="card kpi"><div class="kpi-label">${icon('temp')}最高关节温度</div><div class="kpi-value" id="kpi-temp">—</div><div class="kpi-note" id="kpi-temp-note">反馈温度 · 持续更新</div></div>
  </div>
  <div class="section-gap">${card('作业入口',`<div class="equal-split">${entryColumn(entries.slice(0,3))}${entryColumn(entries.slice(3))}</div>`,'navigation')}</div>
  <div class="split section-gap">
   ${card('运行状态',`<div class="card-pad"><div class="health-list">${kv('运动状态','—','health-motion')}${kv('使用模式','—','health-mode')}${kv('当前步态','—','health-gait')}${kv('地图定位','—','health-loc')}${kv('硬急停','—','health-hes')}${kv('充电状态','—','health-charge')}</div><div class="quick-actions">${cmd('站立','stand')}${cmd('趴下','lie')}${btn('前往驾驶','goto-motion','','motion')}${cmd('停止运动','stop','danger')}</div></div>`,'sensors','<span class="dot"></span>')}
   ${card('当前任务','<div id="overview-task"></div>','tasks',btn('任务中心','goto-tasks','small ghost','arrow'))}
  </div>
  <div class="section-gap">${card('机器人数字视图',stage()+`<div class="source-note" id="overview-source"></div>`,'joints','<span class="pill gray" id="model-pill">等待机身回报</span>')}</div>
  <div class="equal-split section-gap">
   ${card('最近动态','<div class="log-scroll" id="overview-logs"></div>','clock')}
   ${card('计算单元温度',`<canvas id="temp-chart" class="mini-chart" style="width:100%;height:120px;display:block"></canvas><div class="chart-legend"><span>最近 60 秒</span><span><span class="dot"></span>最高主机温度 / °C</span></div>`,'cpu')}
  </div>`;
};
ui['work-sensors']=()=>navigate('sensors');ui['work-diagnostics']=()=>navigate('diagnostics');
let operationHideTimer=null;
const detailedOperation=setOperation;
setOperation=function(label,progress,message,status){
 detailedOperation(label,progress,message,status);clearTimeout(operationHideTimer);
 const panel=$('#hub-operation');if(panel&&!panel.querySelector('[data-ui="hide-operation"]')){
  const close=document.createElement('button');close.type='button';close.className='operation-close';close.dataset.ui='hide-operation';close.textContent='×';close.title='关闭操作提示';panel.append(close);
 }
 if(status==='completed'||!status&&progress>=1)operationHideTimer=setTimeout(()=>panel?.setAttribute('hidden',''),1800);
};
ui['hide-operation']=()=>$('#hub-operation')?.setAttribute('hidden','');

const fullConnectionCenter=connectionCenter;
connectionCenter=function(){
 fullConnectionCenter();if(!connections)return;
 const content=$('#connection-content'),form=$('#connections-profile-form');if(!content||!form)return;
 const share=$('#sharing-panel');
 const hostCard=(role,title,description)=>{
  const target=connections.targets?.[role];
  let actions='';
  if(role==='aos')actions='<button type="button" class="text-button" data-pair-role="aos">重新配对</button>';
  else if(target&&!target.enabled)actions='<button type="button" class="text-button" data-ui="open-connection-advanced">在高级管理中启用</button>';
  else if(target&&!target.deployed)actions=`<button type="button" class="text-button" data-quick-setup="${role}">完成首次连接</button>`;
  else if(target)actions=`<div class="button-row"><button type="button" class="text-button" data-connection-role="${role}" data-connection-operation="connect">连接</button><button type="button" class="text-button" data-connection-role="${role}" data-connection-operation="disconnect">断开</button><button type="button" class="text-button" data-service-role="${role}" data-service-operation="status">服务状态</button></div>`;
  return `<div><strong>${title}<small>${description}</small></strong><span id="quick-host-${role}">等待读取</span>${actions}</div>`;
 };
 const quick=document.createElement('section');quick.className='card quick-connect';
 quick.innerHTML=`<div class="card-pad"><h2>连接机器人 <span class="pill gray" id="connection-state-pill">读取连接状态</span></h2><div class="connection-state" id="connection-summary" hidden></div><div class="quick-connect-form">${field('机器人地址','quick-aos-ip',connections.aos_ip)}${btn('自动查找','discover-robot','','refresh')}${btn('连接机器人','hub-connect','primary','link')}${btn('离线体验','hub-demo')}<button type="button" class="btn small ghost" data-ui="hub-disconnect" id="connection-disconnect" hidden>断开</button></div><p class="help">电脑与机器人接入同一 Wi-Fi。连接后平台自动维护已启用的主机通道；首次安装需要设备密码，密码只用于本次安装。</p></div>`;
 const hosts=document.createElement('section');hosts.className='card section-gap';
 hosts.innerHTML=`<div class="card-header"><h2 class="card-title">主机状态</h2><span class="help">按已启用的通道维护后台连接</span></div><div class="card-pad"><div class="quick-hosts">${hostCard('aos','AOS','运动控制 · UDP 直连')}${hostCard('nos','NOS','建图、定位与导航')}${hostCard('gos','GOS','用户扩展程序')}</div><p class="help">AOS 由 UDP 直连，无需后台通道；NOS / GOS 完成首次安装后可单独连接，服务需单独启动。</p></div>`;
 const advanced=document.createElement('details');advanced.className='connection-advanced optional-services';advanced.id='connection-advanced';
 advanced.innerHTML='<summary>高级管理：账号、服务与网络设置</summary><p class="help card-pad">机器人地址与连接方式见页面上方。这里保存主机账号、运行环境脚本与端口，修改后下次连接生效。</p>';advanced.append(form);
 if(typeof form.querySelectorAll==='function'){
  for(const duplicate of form.querySelectorAll('[data-ui="hub-connect"],[data-ui="hub-demo"]'))duplicate.remove();
  for(const primary of form.querySelectorAll('.btn.primary'))primary.classList.remove('primary');
 }
 content.replaceChildren(quick);
 if(share){const sharing=document.createElement('details');sharing.className='sharing-fold card';sharing.innerHTML='<summary>同一 Wi-Fi 的其他电脑如何使用</summary>';sharing.append(share);content.append(sharing);}
 content.append(hosts);content.append(advanced);
 $('#quick-aos-ip').addEventListener('input',event=>{$('#hub-aos-ip').value=event.target.value;});
 paintQuickConnections();
};
ui['open-connection-advanced']=()=>{const advanced=$('#connection-advanced');if(!advanced)return;advanced.open=true;advanced.scrollIntoView({behavior:'smooth',block:'start'});};
function paintQuickConnections(){
 if(page!=='settings')return;
 const discover=$('[data-ui="discover-robot"]');if(discover)discover.disabled=typeof connectionDiscoveryBusy!=='undefined'&&connectionDiscoveryBusy;
 const demo=S.mode==='demo',online=!!S.online,ip=S.ip||connections?.aos_ip||'机器人地址待配置';
 text('quick-host-aos',demo?'离线演示':online?'已连接':'等待回包');
 for(const role of ['nos','gos'])text('quick-host-'+role,demo?'未连接实机':connectionNames[connections?.targets?.[role]?.state?.status]||'尚未连接');
 const summary=$('#connection-summary');
 if(summary){summary.hidden=false;summary.textContent=demo?'离线演示 · 未连接实机，遥测与地图均为模拟数据。':online?('已连接 '+ip+' · UDP 状态正常；运动控制需在驾驶页重新开启。'):('等待回包 · '+ip+'：请检查电脑与机器人是否在同一网络。');}
 const pill=$('#connection-state-pill');
 if(pill){pill.textContent=demo?'离线演示':online?'已连接':'等待回包';pill.className='pill '+(demo?'amber':online?'green':'gray');}
 const disconnect=$('#connection-disconnect');if(disconnect)disconnect.hidden=demo;
 const panel=$('#sharing-panel');if(panel&&!panel.closest('.sharing-fold')){
  const fold=document.createElement('details');fold.className='sharing-fold card';fold.innerHTML='<summary>同一 Wi-Fi 的其他电脑如何使用</summary>';panel.before(fold);fold.append(panel);
 }
}
const dailyConnectionLoad=loadConnections;
loadConnections=async function(paint=false){await dailyConnectionLoad(paint);if(page==='settings'&&!$('#quick-aos-ip')&&connections)connectionCenter();paintQuickConnections();};
settings=function(){return heading(btn('物联网平台','iot-settings','','link')+btn('导出工作区','export','','download')+btn('设备维护','maintenance-modal','small','settings'))+operationPanel()+'<div id="connection-content"></div>';};

const detailedMapsPage=mapsPage;
let mappingView='cloud';
mapsPage=function(){
 const holder=document.createElement('div');holder.innerHTML=detailedMapsPage();
 const cloud=holder.querySelector('#cloud-view')?.closest('.card')?.outerHTML||'';
 const files=holder.querySelector('#hub-upload-form')?.closest('.card')?.outerHTML||'';
 const gridTools=holder.querySelector('#grid-tools');
 if(gridTools){gridTools.querySelector('#nos-map-name')?.closest('.card')?.remove();gridTools.open=false;}
 const grid=gridTools?.outerHTML||'';
 const dpad=`<div class="mapping-dpad"><button data-velocity="0,0,1" aria-label="建图左转">↶</button><button data-velocity="1,0,0" aria-label="建图前进">↑</button><button data-velocity="0,0,-1" aria-label="建图右转">↷</button><button data-velocity="0,1,0" aria-label="建图左移">←</button><button class="center" data-action="stop">停止</button><button data-velocity="0,-1,0" aria-label="建图右移">→</button><span></span><button data-velocity="-1,0,0" aria-label="建图后退">↓</button><span></span></div>`;
 return heading(btn('同步活动地图','sync-map','','refresh')+btn('原图续建 / 优化','extend-map','','maps')+btn('地图包与导入','map-files-open','','file'))+operationPanel()+
 `<section class="card mapping-phase-card"><div><span id="mapping-phase" class="mapping-phase idle">等待状态</span><strong id="mapping-phase-message">正在读取建图状态</strong><small id="mapping-phase-extra"></small></div></section>
  <div class="mapping-layout">
   <aside class="mapping-sidebar">
    ${card('我的地图','<div class="map-library-toolbar"><input id="map-library-search" type="search" placeholder="搜索地图名称或编号" aria-label="搜索地图"><div><span id="map-library-count" class="help"></span><button class="text-button" data-ui="map-library-trash">回收站</button></div></div><div id="map-batch-bar" class="map-batch-bar" hidden></div><div id="remote-map-list" class="compact-map-library"></div><p id="map-disk" class="help card-pad"></p>','maps',`<div class="button-row"><span id="map-library-source" class="pill ${S.mode==='demo'?'amber':'gray'}">${S.mode==='demo'?'演示 NOS':'NOS'}</span>${btn('编辑地图','map-library-edit','small')}</div>`)}
    <section class="card card-pad selected-map-work"><div><strong id="selected-map-info">请选择一张地图</strong><p class="help">选择地图后可用于导航、导出或继续补扫。</p></div>${select('栅格','remote-grid-id',[[0,'栅格 0']])}<div class="button-row">${btn('用于导航','map-use-navigation','primary','navigation')}</div><h4 class="field-heading">导出与维护</h4><div class="button-row">${btn('导出地图包','export-selected-map','')}${btn('补扫与版本','map-project','')}</div></section>
   </aside>
   <div class="mapping-visuals">
    <section class="card build-controls-card"><div class="card-header"><h2 class="card-title">${icon('maps')}建图采集</h2><span class="pill gray">低速遥控</span></div><div class="build-controls"><div>${field('新地图名称','hub-map-name','m20-map')}<div class="button-row section-gap">${btn('开始建图','hub-mapping-start','primary','play')}${btn('停止并保存','hub-mapping-stop','danger','stop')}</div><p class="help">开始后按住方向键；完成后点击“停止并保存”。</p></div><div>${dpad}</div><div><div class="range-label"><span>遥控幅度</span><b id="speed-label">0.50</b></div><input id="motion-speed" class="range" type="range" min="0.05" max="0.7" step="0.05" value="0.5"><p class="help">松开只停走。可多圈补扫、回到旧区域形成回环。</p></div></div></section>
    <div class="mapping-view-toolbar">
     <div class="segmented">${[['cloud','三维点云'],['grid','二维地图'],['scan','俯视扫描'],['both','对照查看']].map(([key,label])=>`<button class="${mappingView===key?'active':''}" data-scan-view="${key}" aria-pressed="${mappingView===key}">${label}</button>`).join('')}</div>
     <div class="mapping-view-actions">
      <span id="scan-receive-state">选择已保存地图，或在建图时接收实时点云</span>
      ${btn('刷新地图','map-library-refresh','small','refresh')}
     </div>
    </div>
    <div id="mapping-views" class="mapping-views show-${mappingView}"><div class="mapping-cloud">${cloud}</div><section class="card mapping-scan" id="map-scan-section"><div class="card-header"><h2 class="card-title">${icon('sensors')}雷达俯视扫描</h2><span id="scan-frame" class="pill gray">等待数据</span></div><div class="scan-canvas-wrap"><canvas id="mapping-scan"></canvas><p id="scan-empty">开始接收点云后显示 XY 俯视投影</p></div><p class="help card-pad">雷达扫描俯视投影</p></section><section class="card mapping-grid"><div class="card-header"><h2 class="card-title">${icon('maps')}二维栅格地图</h2><span id="map-grid-name" class="pill gray"></span></div><div class="map-grid-preview"><img id="map-grid-image" alt="选中地图的二维栅格" hidden><p id="map-grid-message" class="help" role="status">请从地图库选择地图</p></div></section></div>
   </div>
  </div>
  <details open class="card map-file-tools" id="map-file-tools"><summary>地图包导入、电脑文件与详细定位工具</summary><div class="map-file-tools-content">${files}${grid}<div class="button-row">${btn('地图文件详情','map-details','small')}${btn('本机 drmap 帮助','map-vendor-help','small')}</div></div></details>
  <details open class="card mapping-log-fold"><summary>查看建图日志</summary><div id="mapping-progress" class="card-pad"></div></details>`;
};
ui['map-files-open']=()=>{$('#map-file-tools').open=true;$('#map-file-tools').scrollIntoView({behavior:'smooth'});};
function paintMappingGrid(){
 if(page!=='maps'||mappingView!=='grid')return;
 const image=$('#map-grid-image'),message=$('#map-grid-message');
 if(!image||!message)return;
 const map=mapLibrary.maps.find(row=>row.name===selectedRemote);
 const grid=$('#remote-grid-id')?.value||'0';
 text('map-grid-name',map?`${map.display_name||map.name} · 栅格 ${grid}`:'');
 if(!map||!map.grids.includes(Number(grid))){
  image.hidden=true;delete image.dataset.previewKey;image.removeAttribute('src');
  message.hidden=false;message.textContent=map?'该地图没有可查看的二维栅格':'请从地图库选择地图';return;
 }
 const key=JSON.stringify([navigationMapScope(),map.name,grid,map.modified]);
 if(image.dataset.previewKey===key)return;
 image.dataset.previewKey=key;image.hidden=true;message.hidden=false;message.textContent='正在加载二维地图…';
 image.onload=()=>{if(image.dataset.previewKey===key){image.hidden=false;message.hidden=true;}};
 image.onerror=()=>{if(image.dataset.previewKey===key)message.textContent='二维地图加载失败，请切换视图后重试';};
 image.src='/api/maps/library/preview?'+new URLSearchParams({name:map.name,grid_id:grid});
}
let buildState=null,scanSequence=null,workflowBuildRequest=null;
let scanReceivedAt=0;
const workflowShowCloud=showCloud;
showCloud=function(data,fit=true){scanReceivedAt=Date.now();workflowShowCloud(data,fit);};
loadBuildStatus=async function(){
 const scope=navigationMapScope(),sourcePage=page;
 if(workflowBuildRequest?.scope===scope)return;
 const token={scope};workflowBuildRequest=token;buildLoading=true;const current=()=>workflowBuildRequest===token&&scope===navigationMapScope();
 try{
  const r=await api('/api/maps/build/status',undefined,true);if(!current())return;buildState=r;
  const phase=r.phase||'unconfirmed',labels={idle:'未开始',starting:'启动中',collecting:'正在采集',saving:'停止并保存中',saved:'已保存',interrupted:'采集中断 · 数据保留',empty:'已停止 · 无地图产物',external_collecting:'外部建图任务',unconfirmed:'状态待核对',error:'需要处理'};
  const pill=$('#mapping-phase');if(pill){pill.textContent=(r.demo?'演示 · ':'')+(labels[phase]||phase);pill.className='mapping-phase '+phase;}
  text('mapping-phase-message',r.message||'暂未收到采集状态，请检查 NOS 连接');
  text('mapping-phase-extra',r.result_maps?.length?'结果地图：'+r.result_maps.join('、'):r.fusion?'本轮融合：'+r.fusion.keyframes+' 帧 · 回环 '+r.fusion.loop_constraints+' 个 · 配准误差 '+r.fusion.quality.rmse_m+' m':r.in_progress&&r.mapping?.context?.base?'正在补扫：'+r.mapping.context.base:r.in_progress&&r.mapping?.name?'正在创建：'+r.mapping.name:r.demo?'演示采集 · 无真实雷达数据':'等待采集状态');
  const start=$('[data-ui="hub-mapping-start"]'),stop=$('[data-ui="hub-mapping-stop"]');if(start)start.disabled=!!r.in_progress;if(stop)stop.disabled=!r.in_progress;
  html('mapping-progress',['mapping','stop_mapping'].map((key,i)=>`<strong>${i?'保存日志':'启动日志'}</strong><pre class="pre">${esc(r[key]?.output||r[key]?.message||'暂无日志')}</pre>`).join(''));
  if(r.phase==='saved'&&r.result_maps?.length&&lastBuildEnd!==scope+'|'+r.result_maps.join(',')){
   const previousSelection=selectedRemote;await loadMapLibrary();if(!current()||page!==sourcePage||previousSelection&&selectedRemote!==previousSelection)return;
   if(!mapLibrary.maps.some(m=>m.name===r.result_maps[0]))return;
   lastBuildEnd=scope+'|'+r.result_maps.join(',');selectedRemote=r.result_maps[0];paintMapLibrary();liveCloudTopic=null;
   if(r.fusion&&mapLibrary.active===selectedRemote){try{const synced=await syncNavigationMap(selectedRemote,r.fusion.grid_id||0);if(!current()||!synced||page!==sourcePage)return;points=[];routeId=null;}catch(e){if(!current())return;toast('新版本已保存，导航栅格暂未同步：'+e.message,true);}}
   if(current()&&page==='maps')previewCloud('map',selectedRemote).catch(e=>{if(current())toast('地图已保存，点云预览暂不可用：'+e.message,true);});
  }
 }catch(e){if(current()){text('mapping-phase-message','暂时无法读取建图状态：'+e.message);text('mapping-phase','状态不可读');}}
 finally{if(workflowBuildRequest===token){workflowBuildRequest=null;buildLoading=false;}}
};
ui['map-build-status']=loadBuildStatus;
ui['hub-mapping-start']=async()=>{
 await job('/api/maps/start-here',{name:$('#hub-map-name').value},'停稳并启动建图');await loadBuildStatus();
 if(S.mode==='live'){if($('#live-cloud-topic'))$('#live-cloud-topic').value='/SLAM_ALIGNED_POINTS';ui['cloud-live']().catch(()=>{});}toast(S.mode==='demo'?'演示建图已开始':'启动已提交，请观察采集状态与点云');
};
ui['hub-mapping-stop']=async()=>{
 await job('/api/maps/library/build',{action:'stop'},'停止并保存地图');await loadBuildStatus();await loadMapLibrary();toast(S.mode==='demo'?'演示保存完成':'保存已提交，请等待“已保存”状态');
};
function drawScan(){
 if(cloudSource?.kind==='live'){
  const age=(cloudData?.age||0)+(Date.now()-scanReceivedAt)/1000,fresh=cloudData?.fresh&&age<3;
  text('scan-frame',fresh?'实时扫描':'点云已过期');text('scan-receive-state',fresh?'实时点云 · '+(cloudData.frame_id||'坐标系待提供')+' · '+age.toFixed(1)+' 秒前':'等待新点云，当前保留最后一帧');
 }
 const canvas=$('#mapping-scan');if(!canvas||!canvas.clientWidth||!canvas.clientHeight||!cloudData?.points?.length)return;
 const key=cloudData;if(scanSequence===key&&canvas.width===Math.round(canvas.clientWidth*(devicePixelRatio||1)))return;scanSequence=key;
 const ratio=Math.min(devicePixelRatio||1,2),w=canvas.clientWidth||600,h=canvas.clientHeight||360;canvas.width=w*ratio;canvas.height=h*ratio;
 const ctx=canvas.getContext('2d');ctx.scale(ratio,ratio);ctx.fillStyle='#13272e';ctx.fillRect(0,0,w,h);
 const [lo,hi]=cloudData.bounds,s=Math.min((w-40)/Math.max(.1,hi[0]-lo[0]),(h-40)/Math.max(.1,hi[1]-lo[1]));
 const ox=(w-(hi[0]-lo[0])*s)/2,oy=(h-(hi[1]-lo[1])*s)/2;
 ctx.fillStyle='#68d6b2';const stride=Math.max(1,Math.ceil(cloudData.points.length/20000));
 for(let i=0;i<cloudData.points.length;i+=stride){const p=cloudData.points[i];ctx.fillRect(ox+(p[0]-lo[0])*s,h-oy-(p[1]-lo[1])*s,1.6,1.6);}
 $('#scan-empty').hidden=true;
 if(cloudSource?.kind!=='live'){text('scan-frame','已保存地图');text('scan-receive-state','当前显示已保存的点云地图');}
}

let navIntent='goal';
const originalNavPage=navPage;
function logicalMapKey(map){return JSON.stringify([map.robot_ip||null,map.robot_map_name||map.name,map.grid_id||0,map.origin,map.resolution,map.width,map.height,!!map.demo]);}
mapSelect=function(){
 const groups=new Map();for(const map of S.maps){const key=logicalMapKey(map),old=groups.get(key);if(!old||map.id===mapId||old.id!==mapId&&map.verified_for_robot&&!old.verified_for_robot)groups.set(key,map);}
 return `<div class="field"><label for="map-select">当前编辑地图</label><select id="map-select">${[...groups.values()].map(m=>`<option value="${esc(m.id)}" ${m.id===mapId?'selected':''}>${esc(m.name)}${m.demo?' [演示]':S.mode==='demo'?' [离线缓存]':m.verified_for_robot?' · 已同步':' · 需核对'}</option>`).join('')}</select></div>`;
};
renderRoutes=function(){
 const selected=selectedMap(),key=selected&&logicalMapKey(selected);
 const routes=S.routes.filter(r=>{const m=S.maps.find(m=>m.id===r.map_id);return m&&(page!=='navigation'||logicalMapKey(m)===key);});
 const markup=routes.map(r=>`<div class="route-row"><div><strong>${esc(r.name)}</strong><small>${r.points.length} 个航点 · ${r.options?.pause_at_point?'每点暂停':'自动接续'}${page==='tasks'?' · '+esc(S.maps.find(m=>m.id===r.map_id)?.name):''}</small></div><div><button class="btn small" data-route-load="${esc(r.id)}">编辑</button><button class="btn small primary" data-route-run="${esc(r.id)}">执行</button><button class="btn small ghost" data-route-delete="${esc(r.id)}" aria-label="删除路线">×</button></div></div>`).join('')||empty('常用巡检路线',page==='navigation'&&navIntent==='goal'?'单个目标无需保存；固定巡检任务可切换到多点巡检。':'点击地图添加航点，然后保存路线。','navigation');const list=$('#route-list');if(list&&list._routeMarkup!==markup){list.innerHTML=markup;list._routeMarkup=markup;}
};
function navigationDrivePanel(){
 return `<details class="navigation-drive">
  <summary><span class="navigation-drive-title">${icon('motion')}<strong>手动接管</strong></span><span id="nav-drive-state"></span></summary>
  <div class="navigation-drive-body">
   ${btn('接管驾驶','nav-takeover','small')}
   <div class="mapping-dpad" role="group" aria-label="手动驾驶方向">
    <button data-velocity="0,0,1" aria-label="左转">↶</button>
    <button data-velocity="1,0,0" aria-label="前进">↑</button>
    <button data-velocity="0,0,-1" aria-label="右转">↷</button>
    <button data-velocity="0,1,0" aria-label="左移">←</button>
    <button class="center" data-action="stop">停止</button>
    <button data-velocity="0,-1,0" aria-label="右移">→</button>
    <span></span><button data-velocity="-1,0,0" aria-label="后退">↓</button><span></span>
   </div>
   <div class="navigation-drive-speed">
    <div class="range-label"><label for="motion-speed">遥控幅度</label><output for="motion-speed" id="speed-label">0.50</output></div>
    <input id="motion-speed" class="range" type="range" min="0.05" max="0.7" step="0.05" value="0.5">
   </div>
   <small id="motion-readiness"></small>
  </div>
 </details>`;
}
function navigationMapCard(){
 const placeholder=S.mode==='demo'?'演示模式没有实时画面':S.online?'正在连接前视相机…':'等待机器人连接';
 const actions=`<div class="navigation-map-actions">
  <button type="button" class="btn small" id="nav-camera-toggle" data-ui="nav-camera-toggle" aria-expanded="false" aria-controls="nav-camera-preview">${icon('vision')}前视</button>
  <span class="pill blue" id="nav-status">待命</span>
 </div>`;
 const map=`<div class="navigation-map-stage">
  ${mapStage()}
  <section class="nav-camera-preview" id="nav-camera-preview" aria-label="前视相机" hidden>
   <div class="nav-camera-header"><strong>前视</strong><span id="cam-front-tag"></span>
    <button type="button" data-ui="nav-camera-toggle" aria-label="关闭前视">×</button>
   </div>
   <div class="camera-feed">
    <img id="camera-front" alt="前视相机画面" hidden>
    <div id="placeholder-front" class="camera-placeholder"><strong>${placeholder}</strong></div>
   </div>
  </section>
 </div>`;
 return card('路线地图',map,'maps',actions);
}
function taskCollectionSettings() {
 const saved=S.routes?.find(route=>route.id===routeId)?.options?.collectors||[];
 const types=S.task_collectors||[];
 if(!types.length)return '';
 return `<section id="task-collection-settings" class="route-advanced" aria-labelledby="task-collection-title"><h4 id="task-collection-title" class="field-heading">沿途采集</h4>
 <div class="collector-list">${types.map(type=>{
  const current=saved.find(item=>item.kind===type.kind);
  const fields=type.fields.map(field=>{
   if(field.device_picker){
    const ids=current?.[field.key]||(current?.device_id?[current.device_id]:[]);
    return iotPicker(ids,`data-collector-field="${esc(field.key)}"`,field.label);
   }
   const value=current?.[field.key]??'';
   const input=field.choices
    ? `<select data-collector-field="${esc(field.key)}">${Object.entries(field.choices).map(([key,label])=>
       `<option value="${esc(key)}" ${key===value?'selected':''}>${esc(label)}</option>`).join('')}</select>`
    : `<input data-collector-field="${esc(field.key)}" inputmode="numeric" pattern="${esc(field.pattern)}" value="${esc(value)}">`;
   return `<label class="field">${esc(field.label)}${input}</label>`;
  }).join('');
  return `<div class="collector-row" data-collector="${esc(type.kind)}">
   <label class="check collector-toggle"><input type="checkbox" data-collector-enable ${current?'checked':''}><span>${esc(type.title)}</span></label>
   <div data-collector-options class="form-grid collector-options" ${current?'':'hidden'}>${fields}
    <label class="field">采集间隔（秒）<input data-collector-interval type="number"
     min="${type.min_interval_s}" max="${type.max_interval_s}" step="1" value="${current?.interval_s??type.interval_s}"></label>
   </div></div>`;
 }).join('')}</div></section>`;
}

function taskCollectorOptions() {
 if(!$('#task-collection-settings'))return S.routes?.find(route=>route.id===routeId)?.options?.collectors||[];
 return $$('#task-collection-settings [data-collector]')
  .filter(row=>row.querySelector('[data-collector-enable]').checked)
  .map(row=>({kind:row.dataset.collector,interval_s:+row.querySelector('[data-collector-interval]').value,
              ...Object.fromEntries([...row.querySelectorAll('[data-collector-field]')].map(field=>[field.dataset.collectorField,field.hasAttribute('data-iot-selection')?iotSelected(field):field.value]))}));
}

function navigationIntent(){
 return points.length>1||points.some(point=>point.type===3||point.station_action)||
        points.length===1&&(taskCollectorOptions().length||($('#route-navigation-engine')?.value||'gos')!=='gos'||($('#disconnect-policy')?.value||'stop')!=='stop')?'route':'goal';
}
function navigationActions(single){
 const run=single?btn('执行导航','go-goal','primary','play'):btn('执行巡检','start-nav','primary','play');
 const replan=single&&points.length?btn('重新规划','goal-replan','small'):'';
 return btn('撤销一点','undo-waypoint','small ghost')+btn('保存路线','save-route','','file')+run+replan+btn('暂停','pause-nav')+btn('继续','resume-nav')+btn('结束任务','cancel-nav','danger');
}
function routeEditorPage(){
 const single=navigationIntent()==='goal';
 return heading(btn('任务中心','goto-tasks','','tasks'))+operationPanel()+
 `<div class="route-workspace">
   <div class="route-map-column">
    <div class="route-action-bar" id="nav-action-bar">${navigationActions(single)}</div>
    ${navigationMapCard()}
    <div id="nav-feedback" class="card nav-feedback-empty"></div>
    ${navigationDrivePanel()}
    <div id="patrol-station-results"></div>
   </div>
   <aside class="route-editor card card-pad">
    ${mapSelect().replace('当前编辑地图','导航地图')}
    <p id="navigation-executor-status" class="help" role="status"></p>
    <div class="divider"></div>
    <h3 id="nav-target-title">导航目标</h3>
    <div id="goal-position" class="goal-position">尚未选择目标</div>
    <label class="field">到达朝向<select id="goal-yaw-mode"><option value="auto">沿路径末段方向</option><option value="manual">指定朝向</option></select></label>
    <div id="goal-yaw-field" hidden>${field('到达朝向 / 度','goal-yaw',0,'number')}</div>
    <div id="goal-plan-status" class="goal-plan-status">${single?'在地图上选定目标，自动规划可通行路径':'多个航点将按顺序执行；点击“执行巡检”开始。'}</div>
    <div class="divider"></div>
    ${field('路线名称','route-name',routeId?(S.routes.find(r=>r.id===routeId)?.name||'新路线'):'新路线')}
    <div class="form-grid">
     ${select('寻路方式','route-navigation-engine',[['gos','GOS 机载寻路'],['native','原厂导航']])}
     ${select('电脑断联后','disconnect-policy',[['stop','停车'],['finish','完成任务后停车'],['return','立即沿原路返回'],['finish_return','完成任务后沿原路返回']])}
    </div>
    <p id="disconnect-policy-help" class="help" role="status"></p>
    <label class="field">到点后<select id="route-arrival"><option value="continue">自动前往下一点，终点结束</option><option value="pause">每个任务点暂停，点击继续</option></select></label>
    <div class="range-label"><strong>行进顺序</strong><b id="point-count">${points.length} 个点</b></div>
    <div id="waypoints" class="waypoints"></div>
    <p class="help">在地图上添加航点后，可直接选择每个航点的到点任务。</p>
    ${taskCollectionSettings()}
    <details id="dock-route-settings" class="route-advanced" open ${points.some(p=>p.type===3)?'':'hidden'}><summary>原厂导航参数</summary><div class="form-grid">${select('行进方式','nav-mode',[[1,'自主导航'],[0,'分段直线']])}${select('导航速度','nav-speed',[[1,'低速'],[0,'正常'],[2,'高速']])}${select('路径步态','nav-gait',[['flat','敏捷平地'],['stair','敏捷楼梯']])}${select('运动朝向','nav-manner',[[0,'前进'],[1,'倒退']])}</div></details>
    <div class="divider"></div>
    <h3>已保存路线</h3>
    <div id="route-list"></div>
   </aside>
  </div>`;
}
function refreshNavActions(){
 const bar=$('#nav-action-bar');if(!bar)return;
 navIntent=navigationIntent();
 const single=navIntent==='goal';
 bar.innerHTML=navigationActions(single);
 const title=$('#nav-target-title');if(title)title.textContent=single?'导航目标':'巡检航点';
 const status=$('#goal-plan-status');if(status&&!single)status.textContent='按顺序前往航点，点击“执行巡检”开始。';
}
navPage=function(){
 mapTool='waypoint';
 const holder=document.createElement('div');holder.innerHTML=routeEditorPage();
 return holder.innerHTML;
};
ui['nav-takeover']=async()=>{await job('/api/nav/manual',{},'暂停路线并切换手动');await fetchState();toast('已切换手动，按住方向键移动');};
ui['undo-waypoint']=()=>{points.pop();renderPoints();};
function routeOptions(){
 return {nav_mode:+($('#nav-mode')?.value??1),speed:+($('#nav-speed')?.value??1),gait:$('#nav-gait')?.value||'flat',manner:+($('#nav-manner')?.value??0),
  pause_at_point:$('#route-arrival')?.value==='pause',collectors:taskCollectorOptions(),
  engine:points.some(point=>point.type===3)?'native':$('#route-navigation-engine')?.value||'gos',
  disconnect_policy:$('#disconnect-policy')?.value||'stop',explicit_execution:true};
}
saveRoute=async function(){
 const r=await api('/api/route/save',{id:routeId,name:$('#route-name')?.value||'新巡检路线',map_id:mapId,points,options:routeOptions()});routeId=r.route.id;await fetchState();return r.route;
};
runRoute=async function(id){
 const r=S.routes.find(r=>r.id===id),options=$('#nav-mode')&&r?.id===routeId?routeOptions():r?.options||{nav_mode:1,speed:1,gait:'flat',manner:0,pause_at_point:false};
 await api('/api/nav/start',{id,...options});await fetchState();toast(S.mode==='demo'?'演示路线已开始':'路线已提交，等待导航反馈');
};
const classicRenderPoints=renderPoints;
function stationFields(index, point) {
 const action = point.station_action || {};
 const camera = action.camera || 'none';
 const options = [['none','不统计'],['front','前视相机'],['rear','后视相机'],['ball','外接球机']];
 const devices=action.iot_device_ids||(action.iot_device_id?[action.iot_device_id]:[]);
 const row = document.createElement('div');
 row.className = 'stack waypoint-station';
 row.innerHTML = `<label class="field">到点人数统计<select data-station-point="${index}" data-station-key="camera">
   ${options.map(([value,label])=>`<option value="${value}" ${value===camera?'selected':''}>${label}</option>`).join('')}
 </select></label>
   <label class="field station-pan" ${camera==='ball'?'':'hidden'}>云台动作
     <select data-station-point="${index}" data-station-key="pan">
       <option value="none">当前位置</option><option value="left" ${action.pan==='left'?'selected':''}>短转向左</option>
       <option value="right" ${action.pan==='right'?'selected':''}>短转向右</option>
     </select>
   </label>
   ${iotPicker(devices,`data-station-point="${index}" data-station-key="iot_device_ids"`,'到点读取设备')}`;
 row.addEventListener('change', event=>{
  if (!event.target.dataset.stationKey) return;
  const camera=row.querySelector('[data-station-key="camera"]').value;
  const pan=row.querySelector('[data-station-key="pan"]');
  const devices=iotSelected(row.querySelector('[data-iot-selection]'));
  if(camera!=='ball')pan.value='none';
  row.querySelector('.station-pan').hidden=camera!=='ball';
  if(camera==='none'&&!devices.length)delete point.station_action;
  else point.station_action={camera,pan:pan.value,iot_device_ids:devices};
 });
 return row;
}

renderPoints=function(){
 if(page==='navigation')refreshNavActions();
 classicRenderPoints();text('goal-position',points.length?'目标：X '+fmt(points[0].x,2)+' m · Y '+fmt(points[0].y,2)+' m':'尚未选择目标');
 for(const row of $$('#waypoints .waypoint')){
  const type=row.querySelector('select[data-point]'),index=+type?.dataset.point;if(!Number.isFinite(index))continue;
  for(const input of row.querySelectorAll('input[data-point]'))input.remove();
  const typeField=document.createElement('label');typeField.className='field waypoint-type';typeField.textContent='航点类型';typeField.append(type);
  const summary=document.createElement('span');summary.className='point-position';summary.textContent='航点 '+(index+1)+' · X '+fmt(points[index].x,2)+' / Y '+fmt(points[index].y,2);
  row.querySelector('button')?.before(summary);row.append(typeField,stationFields(index,points[index]));row.classList.add('waypoint-simple');
 }
};
document.addEventListener('change',event=>{
 if(page==='navigation'&&event.target.matches('[data-point][data-key="type"],[data-station-key="camera"],[data-station-key="iot_device_ids"],[data-collector-enable],#route-navigation-engine,#disconnect-policy')){
  refreshNavActions();paintGoalPlan();
 }
});

const dayRender=render,dayUpdate=update;
const detailedTaskSummary=taskSummary;
function collectionDownloadButton(task) {
 const id=task?.collection_run?.id;
 return id&&task.collection?.some(row=>row.samples||row.failures)
  ? `<button type="button" class="btn small" data-collection-download="${esc(id)}">下载采集记录</button>` : '';
}
function collectionResults(task) {
 const rows=task?.collection||[];
 if(!rows.length)return '';
 const types=S.task_collectors||[];
 const content=rows.map(row=>{
  const label=types.find(type=>type.kind===row.spec?.kind)?.title||'采集项';
  const values=row.latest?.values;
  const devices=row.latest?.devices||[];
  const summary=row.spec?.kind==='people_count'&&values
   ? `最近一帧 ${esc(values.people)} 人 · 峰值 ${esc(row.peak_people)} 人`
   : devices.length?`本轮 ${devices.length} 台设备`:Object.entries(values||{}).slice(0,4).map(([key,value])=>esc(key)+' '+esc(value)).join(' · ');
  const state={waiting:'等待行进',collecting:'采集中',ended:'已结束',error:'采集异常'}[row.state]||'等待采集';
  return `<div class="route-row"><div><strong>${esc(label)} · ${state}</strong>
   <small>${summary||'等待首条读数'} · ${esc(row.samples)} 条记录</small>
   ${iotReadings(devices)}${row.error?`<small>${esc(row.error)}</small>`:''}</div></div>`;
 }).join('');
 return card('沿途采集',`<div class="card-pad">${content}${collectionDownloadButton(task)}</div>`,'vision');
}

function patrolResults(nav) {
 const rows = nav?.station_results || [];
 if (!rows.length) return collectionResults(nav);
 const content = rows.map(row=>{
  const camera = {front:'前视',rear:'后视',ball:'外接球机'}[row.camera] || '相机';
  const readings = Object.entries(row.measurements || {}).map(([name,value])=>esc(name)+' '+esc(value)).join(' · ');
  const counted=Number.isFinite(row.people);
  return `<div class="route-row"><div><strong>航点 ${esc(row.index+1)} · ${counted?esc(row.people)+' 人':'设备读数'}</strong>
    ${counted?`<small>${camera} · 最高同帧人数${readings?' · '+readings:''}</small>`:''}
    ${iotReadings(row.devices||[])}
    ${row.iot_error?`<small>${esc(row.iot_error)}</small>`:''}</div></div>`;
 }).join('');
 return card('到点作业',`<div class="card-pad">${content}</div>`,'vision')+collectionResults(nav);
}

taskSummary=function(){
 const nav=S?.nav,display=typeof navigationPresentation==='function'?navigationPresentation(nav):null;
 if(display)return `<div class="task-summary"><strong>${esc(display.title)}</strong><p>${esc(display.detail)}</p>${nav.executor==='gos'?'':btn('查看执行诊断','native-nav-diagnostics','small','code')}</div>`+patrolResults(nav);
 if(!nav?.route)return detailedTaskSummary();
 const total=nav.route.points.length;
 if(nav.stop_reason==='route_complete'&&nav.status==='cancelling')return `<div class="task-summary"><strong>所有航点已完成，正在结束导航</strong><p>${nav.completed} / ${total} 个航点 · 等待停止请求确认</p></div>`;
 if(nav.stop_reason==='waypoint_pause'&&nav.status==='paused')return `<div class="task-summary"><strong>已到达 ${nav.completed} 号点，任务已暂停</strong><p>点击继续，前往 ${nav.index+1} 号点 · 共 ${total} 个点</p></div>`;
 return detailedTaskSummary();
};
render=function(){
 delete document.body.dataset.page;document.body.dataset.currentPage=page;dayRender();
 if(page==='navigation'){
  const options=S.routes.find(r=>r.id===routeId)?.options||{nav_mode:1,speed:1,gait:'flat',manner:0,pause_at_point:false};
  for(const [id,key,fallback] of [['nav-mode','nav_mode',1],['nav-speed','speed',1],['nav-gait','gait','flat'],['nav-manner','manner',0]])if($('#'+id))$('#'+id).value=String(options[key]??fallback);
  if($('#route-arrival'))$('#route-arrival').value=options.pause_at_point?'pause':'continue';
  const native=options.explicit_execution&&options.engine==='native';
  if($('#route-navigation-engine'))$('#route-navigation-engine').value=native?'native':'gos';
  if($('#disconnect-policy'))$('#disconnect-policy').value=native?options.disconnect_policy||'stop':'stop';
  refreshNavActions();paintGoalPlan();
 }
 if(page==='maps'){scanSequence=null;drawScan();}
};
update=function(){
 dayUpdate();paintQuickConnections();if(page==='maps')drawScan();
 const stopNotice=typeof previousStopPresentation==='function'?previousStopPresentation(S?.previous_connection_stop):null;
 let stopBanner=$('#previous-stop-warning');
 if(stopNotice){if(!stopBanner){stopBanner=document.createElement('div');stopBanner.id='previous-stop-warning';stopBanner.className='alert danger section-gap';stopBanner.setAttribute('role','status');$('#main .page-heading')?.after(stopBanner);}stopBanner.textContent=stopNotice.title+'：'+stopNotice.detail;}
 else stopBanner?.remove();
 const ready=S?.mode==='demo'||S?.online&&S.basic.HES===0&&S.basic.MotionState===17&&[0,2].includes(S.basic.ControlUsageMode)&&!S.sdk.armed;
 const reason=S?.mode==='demo'?'离线演示 · 不连接机器人':!S?.online?'等待连接机器人':S.basic.HES!==0?'请检查实体急停':S.basic.MotionState!==17?'请先站立并等待就绪':![0,2].includes(S.basic.ControlUsageMode)?'请先切换手动模式':'可以遥控';
 const navStatus=page==='navigation'?S.nav.status:null;
 const running=navStatus==='running',stopping=navStatus==='cancelling';
 text('motion-readiness',ready&&S.mode!=='demo'?reason+' · '+({4097:'标准基础',4098:'兼容高台',4099:'标准楼梯',12290:'敏捷平地',12291:'敏捷楼梯'}[S.basic.Gait]||'步态待反馈')+(S.basic.ControlUsageMode===2?' · 避障'+(['未启动','空闲','未触发','正在介入'][S.basic.OOA]||'状态未知'):''):reason);
 for(const button of $$('[data-velocity]'))button.disabled=!ready||running||stopping;
 if(S?.nav.stop_reason==='route_complete'&&S.nav.status==='cancelling')text('nav-status','终点已到达 · 正在停止');
 if(S?.nav.stop_reason==='waypoint_pause'&&S.nav.status==='paused')text('nav-status','任务点已到达 · 等待继续');
 const navDisplay=typeof navigationFeedback==='function'?navigationFeedback(S?.nav):null;if(navDisplay){text('nav-status',navDisplay.label);html('nav-feedback',navDisplay.html);}
 if(page==='navigation'){
  const offline=S.mode==='live'&&!S.online;
  text('nav-drive-state',offline?'实机离线':stopping?'正在停车':running?'导航执行中':navStatus==='paused'?'导航已暂停':S.mode==='demo'?'演示模式':'');
  if(!offline&&(running||stopping))text('motion-readiness',stopping?'等待停车确认':'接管后可手动驾驶');
  const takeover=$('[data-ui="nav-takeover"]');
  if(takeover)takeover.disabled=offline||stopping||Boolean(S.background_operation);
  const hasData=task=>task?.station_results?.length||task?.collection?.length;
  const latest=hasData(S.nav)?S.nav:[...(S.task_history||[])].reverse().find(hasData);
  html('patrol-station-results',patrolResults(latest));
 }
 const pause=$('[data-ui="pause-nav"]'),resume=$('[data-ui="resume-nav"]');if(pause)pause.disabled=S?.nav.status!=='running';if(resume)resume.disabled=S?.nav.status!=='paused';
};
document.addEventListener('change',event=>{
 if(!event.target.matches('[data-collector-enable]'))return;
 event.target.closest('[data-collector]').querySelector('[data-collector-options]').hidden=!event.target.checked;
});
document.addEventListener('click',async event=>{
 const results=event.target.closest('[data-patrol-results]');
 if(results){
  const task=(S.task_history||[]).find(item=>String(item.started)===results.dataset.patrolResults);
  if(task)modal('巡检结果',patrolResults(task));
  return;
 }
 const button=event.target.closest('[data-collection-download]');
 if(!button||button.disabled)return;
 button.disabled=true;
 const run_id=button.dataset.collectionDownload,rows=[];
 try{
  let offset=0;
  while(offset!==null){
   const page=await api('/api/tasks/collection?run_id='+encodeURIComponent(run_id)+'&offset='+offset,undefined,true);
   rows.push(...page.rows);
   if(page.next_offset!==null&&(!Number.isInteger(page.next_offset)||page.next_offset<=offset))throw Error('采集记录页码无效');
   offset=page.next_offset;
  }
  const url=URL.createObjectURL(new Blob([JSON.stringify({run_id,rows},null,2)],{type:'application/json'}));
  const link=document.createElement('a');
  link.href=url;link.download='巡检采集-'+run_id.slice(0,8)+'.json';link.click();
  setTimeout(()=>URL.revokeObjectURL(url),1000);
 }catch(error){toast(error.message,true);}
 finally{button.disabled=false;}
});
document.addEventListener('click',async event=>{
 const setup=event.target.closest('[data-quick-setup]');if(setup){$('#connection-advanced').open=true;setupWizard(setup.dataset.quickSetup).catch(error=>toast(error.message,true));}
 const view=event.target.closest('[data-scan-view]');if(view){
  const previous=mappingView;mappingView=view.dataset.scanView;
  $('#mapping-views').className='mapping-views show-'+mappingView;
  $$('[data-scan-view]').forEach(button=>{button.classList.toggle('active',button===view);button.setAttribute('aria-pressed',String(button===view));});
  if(mappingView==='grid'){delete $('#map-grid-image').dataset.previewKey;paintMappingGrid();}
  else if(previous==='grid'&&selectedRemote&&mappingView!=='scan')await previewCloud('map',selectedRemote);
  scanSequence=null;drawScan();
 }
});
document.addEventListener('change',event=>{if(event.target.id==='remote-grid-id')paintMappingGrid();});

// One entry from driving, navigation and the map library.
let workflowStartMapIntent=null,workflowExtensionIntent=null;
function workflowMapDialogCurrent(token){return token&&token.scope===navigationMapScope()&&token.page===page&&(token.dialog===null||typeof mapDialogGeneration==='undefined'||token.dialog===mapDialogGeneration);}
function workflowMapDialogToken(){return {scope:navigationMapScope(),page,dialog:typeof mapDialogGeneration==='number'?mapDialogGeneration:null};}
function freshMapName(){return 'm20-map-'+new Date().toLocaleString('sv-SE',{hour12:false}).replace(/[^0-9]/g,'');}
ui['start-map-here']=async()=>{
 if(S.mapping){navigate('maps');return;}
 const scope=navigationMapScope();if(workflowStartMapIntent?.scope===scope)return;
 const token={scope,page};workflowStartMapIntent=token;
 const current=()=>workflowStartMapIntent===token&&scope===navigationMapScope();
 try{
  const result=await job('/api/maps/start-here',{name:freshMapName()},'停稳并开始建图',current);
  if(!result||!current()||page!==token.page)return;
  navigate('maps');await loadBuildStatus();
  if(current()&&page==='maps'&&S.mode==='live'){if($('#live-cloud-topic'))$('#live-cloud-topic').value='/SLAM_ALIGNED_POINTS';ui['cloud-live']().catch(()=>{});}
 }finally{if(workflowStartMapIntent===token)workflowStartMapIntent=null;}
};
ui['extend-map']=async()=>{
 const scope=navigationMapScope(),sourcePage=page,selectedBefore=selectedRemote;await loadMapLibrary();if(scope!==navigationMapScope()||page!==sourcePage||selectedBefore&&selectedRemote!==selectedBefore)return;
 const base=page==='navigation'?selectedMap()?.robot_map_name:selectedRemote||mapLibrary.active;
 let ready=S.mode==='demo',message='';
 if(!ready){try{const capability=await api('/api/maps/project?name='+encodeURIComponent(base||mapLibrary.active),undefined,true);ready=capability.extension_ready===true&&capability.pipeline_version>=2;message=capability.capability_message||'当前 NOS 服务尚不支持原图续建，请更新服务。';}catch(e){message='机器人续建组件暂未就绪：'+e.message;}}
 if(scope!==navigationMapScope()||page!==sourcePage)return;
 if(!ready){modal('原图续建暂不可用',`<p>${esc(message)}</p><p>请在高级连接设置中更新 NOS 服务后重试。</p>`,[{label:'打开连接中心',ui:'goto-connections'}]);return;}
 modal('在原图上续建与优化',select('要续建的地图','extension-base',mapLibrary.maps.map(m=>[m.name,m.name]))+'<div class="extension-note"><strong>保留旧区域，只补扫新增或需要改善的区域</strong><p>选择要更新的地图，从已知区域开始补扫。完成后点击“停止并保存”，生成包含旧区域的新版本。</p></div><p class="help">暂停行走不会结束采集。扫描无法对齐时会保留采集数据，原地图保持不变。</p>',[{label:'开始续建',ui:'extension-start'}]);
 if(base)$('#extension-base').value=base;workflowExtensionIntent=workflowMapDialogToken();
};
ui['extension-start']=async()=>{
 const token=workflowExtensionIntent;if(!workflowMapDialogCurrent(token))throw Error('连接或确认窗口已变化，请重新选择续建地图');workflowExtensionIntent=null;
 const base=$('#extension-base').value;const grid=page==='navigation'?(selectedMap()?.grid_id||0):+($('#remote-grid-id')?.value||0);$('#modal').close();
 const result=await job('/api/maps/extend-here',{base,grid_id:grid},'停稳并开始原图续建',()=>token.scope===navigationMapScope());if(!result||token.scope!==navigationMapScope())return;navigate('maps');await loadBuildStatus();if(token.scope===navigationMapScope()&&S.mode==='live'){if($('#live-cloud-topic'))$('#live-cloud-topic').value='/SLAM_ALIGNED_POINTS';ui['cloud-live']().catch(()=>{});}
};
const workflowMotionUpdate=update;
update=function(){workflowMotionUpdate();const names={4097:'标准 · 基础',4098:'兼容 · 高台',4099:'标准 · 楼梯',12290:'敏捷 · 平地',12291:'敏捷 · 楼梯'};
 text('drive-mode-state','当前：'+(['常规 / 手动','自主导航','辅助 / 避障'][S?.basic.ControlUsageMode]||'等待反馈')+' · '+(names[S?.basic.Gait]||'步态待反馈'));
 for(const [key,value] of Object.entries({basic:4097,platform:4098,standard_stair:4099,flat:12290,stair:12291}))$('[data-action="gait_'+key+'"]')?.classList.toggle('active',S?.basic.Gait===value);
};

ui['map-project']=async()=>{
 const name=selectedRemote||mapLibrary.active;if(!name)throw Error('请先选择地图');
 const data=await api('/api/maps/project?name='+encodeURIComponent(name)),project=data.project;
 const rows=(project.sessions||[]).map((r,i)=>`<div class="card card-pad section-gap"><strong>第 ${i+1} 轮补扫 · ${esc(r.map)}</strong><p>${esc(r.keyframes)} 帧 · 本轮回环 ${esc(r.loop_constraints)} 个 · 新覆盖 ${esc(r.new_known_cells)} 个栅格</p><p class="help">配准重叠 ${fmt((r.quality?.overlap||0)*100,1)}% · 匹配残差 ${fmt(r.quality?.rmse_m,3)} m</p></div>`).join('');
 modal('补扫与地图版本',`<p>当前版本：${esc(project.revision)} · 初始地图：${esc(project.root_map)}</p>${rows||'<p class="help">尚无补扫版本。</p>'}<p class="help">旧版本保留在地图库中，可随时切换使用。匹配残差用于核对新旧扫描对齐情况，不是地图绝对精度。</p>`);
};

ui['native-nav-diagnostics']=async()=>{
 if(S.nav?.engine==='platform'){
  const n=S.nav,c=n.controller||{};
  modal('平台寻路执行诊断',`<p>${esc(n.message||'等待轨迹')}</p><p>距目标 ${esc(n.distance_remaining??'—')} 米 · 重规划 ${n.replans||0} 次<br>可用轨迹 ${c.valid_candidates??'—'} / ${c.candidate_count??'—'} · 计算 ${c.compute_ms??'—'} ms</p><p class="help">归一化指令不是实际速度，预测增益不是实机标定。${S.mode==='demo'?'当前为本地演示。':''}</p><details open><summary>完整诊断字段</summary><pre>${esc(JSON.stringify({controller:c,progress:n.progress_detail,plan:n.plan?.diagnostics},null,2))}</pre></details>`);return;
 }
 const r=await api('/api/nos/navigation-diagnostics',undefined,true);const n=r.current;modal('导航执行诊断',`<p>${esc(r.message||'等待诊断')}</p>${n?`<p>原厂模式：${esc(n.mode||'—')}<br>全局路径：${n.path_points} 点<br>日志输出：前进 ${fmt(n.command.forward,3)} / 侧移 ${fmt(n.command.side,3)} / 转向 ${fmt(n.command.yaw,3)}<br>原厂报告的停障距离：${fmt(n.reported_stop_distance_m,2)} m</p><p class="help">${esc(r.boundary||'')} ${r.fresh===false?'日志当前未更新，不能当成实时状态。':''}</p>`:''}<details open><summary>完整诊断字段</summary><pre>${esc(JSON.stringify(r,null,2))}</pre></details>`);
};
