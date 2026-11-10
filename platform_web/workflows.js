'use strict';

// Page copy for the primary work pages; module/tab navigation is rendered by app.js.
Object.assign(pages,{
 overview:['概览','OVERVIEW','设备连接、运行状态与当前任务的统一视图。'],
 motion:['手动驾驶','DRIVE','相机画面、底盘遥控与步态模式。'],
 maps:['建图与地图','MAPPING','建图采集、地图库与点云。'],
 navigation:['导航与巡检','NAVIGATION','单点导航任务与多航点巡检路线的规划与执行。']
});
overview=function(){
 const demo=S.mode==='demo';
 const entry=(key,title,tip)=>`<button type="button" data-page="${key}"><span class="q-icon">${icon(key)}</span><span><strong>${title}</strong><small>${tip}</small></span><span class="arrow">›</span></button>`;
 return heading(btn('导出状态','export','','download')+btn('连接机器人','connect-modal','primary','link'))+
 `<div class="kpi-grid">
   <div class="card kpi"><div class="kpi-label">${icon('link')}设备连接</div><div class="kpi-value" id="kpi-online">在线<small>DEMO</small></div><div class="kpi-note"><span class="dot"></span><span id="kpi-ip">本地演示环境</span></div></div>
   <div class="card kpi"><div class="kpi-label">${icon('battery')}双电池电量</div><div class="kpi-value" id="kpi-battery">—</div><div class="kpi-note" id="kpi-battery-note">前侧 / 后侧电池</div><svg class="kpi-graph" viewBox="0 0 100 40"><path d="M1 31L15 24 28 27 40 17 53 20 66 13 81 15 99 6" fill="none" stroke="#70bfa5" stroke-width="1.5"/></svg></div>
   <div class="card kpi"><div class="kpi-label">${icon('joints')}关节状态</div><div class="kpi-value" id="kpi-joints">—</div><div class="kpi-note">16 个电机 · 四条腿</div></div>
   <div class="card kpi"><div class="kpi-label">${icon('temp')}最高关节温度</div><div class="kpi-value" id="kpi-temp">—</div><div class="kpi-note" id="kpi-temp-note">反馈温度 · 持续更新</div></div>
  </div>
  <div class="overview-main">
   ${card('机器人数字视图',stage()+`<div class="source-note" id="overview-source">演示数据 · 三维姿态随模拟关节变化。</div>`,'joints','<span class="pill" id="model-pill">模拟反馈</span>')}
   <aside class="overview-side">
    ${card('运行状态',`<div class="card-pad"><div class="health-list">${kv('运动状态','—','health-motion')}${kv('使用模式','—','health-mode')}${kv('当前步态','—','health-gait')}${kv('地图定位','—','health-loc')}${kv('硬急停','—','health-hes')}${kv('充电状态','—','health-charge')}</div><div class="quick-actions">${cmd('站立','stand')}${cmd('趴下','lie')}${btn('前往驾驶','goto-motion','','motion')}${cmd('停止运动','stop','danger')}</div></div>`,'sensors','<span class="dot"></span>')}
    ${card('作业入口',`<div class="quick-entry">${entry('motion','手动驾驶','相机、遥控与步态模式')}${entry('navigation','导航与巡检','单点导航与多航点巡检')}${entry('maps','建图与地图','建图采集、地图库与点云')}${entry('autonomy','自主作业','自主探索与人员跟随')}${entry('vision','视觉识别','相机预览、人员与 PPE 检测')}${entry('diagnostics','故障与日志','错误码与原始回包排查')}</div>`,'navigation')}
   </aside>
  </div>
  <div class="overview-bottom">
   ${card('计算单元温度',`<canvas id="temp-chart" class="mini-chart"></canvas><div class="chart-legend"><span>最近 60 秒</span><span><span class="dot"></span>最高主机温度 / °C</span></div>`,'cpu')}
   ${card('当前任务',`<div id="overview-task"></div>`,'tasks',btn('任务中心','goto-tasks','small ghost','arrow'))}
   ${card('最近动态',`<div class="log-scroll" id="overview-logs"></div>`,'clock')}
  </div>`;
};
ui['work-sensors']=()=>navigate('sensors');ui['work-diagnostics']=()=>navigate('diagnostics');
let operationHideTimer=null;
const detailedOperation=setOperation;
setOperation=function(label,progress,message){
 detailedOperation(label,progress,message);clearTimeout(operationHideTimer);
 const panel=$('#hub-operation');if(panel&&!panel.querySelector('[data-ui="hide-operation"]')){
  const close=document.createElement('button');close.type='button';close.className='operation-close';close.dataset.ui='hide-operation';close.textContent='×';close.title='关闭操作提示';panel.append(close);
 }
 if(progress>=1)operationHideTimer=setTimeout(()=>panel?.setAttribute('hidden',''),1800);
};
ui['hide-operation']=()=>$('#hub-operation')?.setAttribute('hidden','');

const fullConnectionCenter=connectionCenter;
connectionCenter=function(){
 fullConnectionCenter();if(!connections)return;
 const content=$('#connection-content'),form=$('#connections-profile-form');if(!content||!form)return;
 const share=$('#sharing-panel');
 const advanced=document.createElement('details');advanced.className='connection-advanced card';advanced.id='connection-advanced';
 advanced.innerHTML='<summary>高级管理：账号、服务与网络设置</summary>';advanced.append(form);
 const quick=document.createElement('section');quick.className='card card-pad quick-connect';
 quick.innerHTML=`<div><h2>连接机器人</h2><p class="help">电脑与机器人接入同一 Wi-Fi。换网后可自动查找已配对的机器人。</p></div><div class="quick-connect-form">${field('机器人地址','quick-aos-ip',connections.aos_ip)}${btn('自动查找','discover-robot','','refresh')}${btn('连接机器人','hub-connect','primary','link')}${btn('离线体验','hub-demo')}</div><div class="quick-hosts">${[['aos','AOS','运动控制'],['nos','NOS','建图导航'],['gos','GOS','扩展功能']].map(([role,title,description])=>`<div><strong>${title}<small>${description}</small></strong><span id="quick-host-${role}">等待读取</span>${role==='aos'||connections.targets[role].deployed?'':`<button class="text-button" data-quick-setup="${role}">完成首次连接</button>`}</div>`).join('')}</div><p class="help">首次安装可能需要设备密码；之后直接连接，平台会自动处理内部主机连接。</p>`;
 content.replaceChildren(quick);
 if(share){const sharing=document.createElement('details');sharing.className='sharing-fold card';sharing.innerHTML='<summary>同一 Wi-Fi 的其他电脑如何使用</summary>';sharing.append(share);content.append(sharing);}
 content.append(advanced);
 $('#quick-aos-ip').addEventListener('input',event=>{$('#hub-aos-ip').value=event.target.value;});
 paintQuickConnections();
};
function paintQuickConnections(){
 if(page!=='settings')return;
 const discover=$('[data-ui="discover-robot"]');if(discover)discover.disabled=typeof connectionDiscoveryBusy!=='undefined'&&connectionDiscoveryBusy;
 text('quick-host-aos',S.mode==='demo'?'离线演示':S.online?'已连接':'等待回包');
 for(const role of ['nos','gos'])text('quick-host-'+role,S.mode==='demo'?'未连接实机':connectionNames[connections?.targets[role]?.state.status]||'尚未连接');
 const panel=$('#sharing-panel');if(panel&&!panel.closest('.sharing-fold')){
  const fold=document.createElement('details');fold.className='sharing-fold card';fold.innerHTML='<summary>同一 Wi-Fi 的其他电脑如何使用</summary>';panel.before(fold);fold.append(panel);
 }
}
const dailyConnectionLoad=loadConnections;
loadConnections=async function(paint=false){await dailyConnectionLoad(paint);if(page==='settings'&&!$('#quick-aos-ip')&&connections)connectionCenter();paintQuickConnections();};
settings=function(){return heading(btn('导出工作区','export','','download')+btn('设备维护','maintenance-modal','small','settings'))+operationPanel()+'<div id="connection-content"></div>';};

const detailedMapsPage=mapsPage;
mapsPage=function(){
 const holder=document.createElement('div');holder.innerHTML=detailedMapsPage();
 const cloud=holder.querySelector('#cloud-view')?.closest('.card')?.outerHTML||'';
 const files=holder.querySelector('#hub-upload-form')?.closest('.card')?.outerHTML||'';
 const grid=holder.querySelector('#grid-tools')?.outerHTML||'';
 const dpad=`<div class="mapping-dpad"><button data-velocity="0,0,1" aria-label="建图左转">↶</button><button data-velocity="1,0,0" aria-label="建图前进">↑</button><button data-velocity="0,0,-1" aria-label="建图右转">↷</button><button data-velocity="0,1,0" aria-label="建图左移">←</button><button class="center" data-action="stop">停止</button><button data-velocity="0,-1,0" aria-label="建图右移">→</button><span></span><button data-velocity="-1,0,0" aria-label="建图后退">↓</button><span></span></div>`;
 return heading(btn('同步活动地图','sync-map','','refresh')+btn('原图续建 / 优化','extend-map','','maps')+btn('地图包与导入','map-files-open','','file'))+operationPanel()+
 `<section class="card mapping-phase-card"><div><span id="mapping-phase" class="mapping-phase idle">等待状态</span><strong id="mapping-phase-message">正在读取建图状态</strong><small id="mapping-phase-extra"></small></div></section>
  <div class="mapping-layout">
   <aside class="mapping-sidebar">
    ${card('我的地图','<div class="map-library-toolbar"><input id="map-library-search" type="search" placeholder="搜索地图名称或编号" aria-label="搜索地图"><div><span id="map-library-count" class="help"></span><button class="text-button" data-ui="map-library-trash">回收站</button></div></div><div id="map-batch-bar" class="map-batch-bar" hidden></div><div id="remote-map-list" class="compact-map-library"></div><p id="map-disk" class="help card-pad"></p>','maps',`<div class="button-row"><span id="map-library-source" class="pill ${S.mode==='demo'?'amber':'gray'}">${S.mode==='demo'?'演示 NOS':'NOS'}</span>${btn('编辑地图','map-library-edit','small')}</div>`)}
    <section class="card card-pad selected-map-work"><div><strong id="selected-map-info">请选择一张地图</strong><p class="help">选择地图后可用于导航、导出或继续补扫。</p></div>${select('栅格','remote-grid-id',[[0,'栅格 0']])}<div class="button-row">${btn('用于导航','map-use-navigation','primary','navigation')}${btn('导出地图包','export-selected-map','','download')}${btn('查看二维地图','sync-selected-map','small')}${btn('补扫与版本','map-project','small')}${btn('刷新地图','map-library-refresh','small')}</div></section>
   </aside>
   <div class="mapping-visuals">
    <section class="card build-controls-card"><div class="card-header"><h2 class="card-title">${icon('maps')}建图采集</h2><span class="pill gray">低速遥控</span></div><div class="build-controls"><div>${field('新地图名称','hub-map-name','m20-map')}<div class="button-row section-gap">${btn('开始建图','hub-mapping-start','primary','play')}${btn('停止并保存','hub-mapping-stop','danger','stop')}</div><p class="help">开始后按住方向键；完成后点击“停止并保存”。</p></div><div>${dpad}</div><div><div class="range-label"><span>遥控幅度</span><b id="speed-label">0.10</b></div><input id="motion-speed" class="range" type="range" min="0.05" max="0.25" step="0.05" value="0.1"><p class="help">松开只停走。可多圈补扫、回到旧区域形成回环。</p></div></div></section>
    <div class="mapping-view-toolbar"><div class="segmented"><button class="active" data-scan-view="cloud">三维点云</button><button data-scan-view="scan">俯视扫描</button><button data-scan-view="both">对照查看</button></div><span id="scan-receive-state">选择已保存地图，或在建图时接收实时点云</span></div>
    <div id="mapping-views" class="mapping-views show-cloud"><div class="mapping-cloud">${cloud}</div><section class="card mapping-scan" id="map-scan-section"><div class="card-header"><h2 class="card-title">${icon('sensors')}雷达俯视扫描</h2><span id="scan-frame" class="pill gray">等待数据</span></div><div class="scan-canvas-wrap"><canvas id="mapping-scan"></canvas><p id="scan-empty">开始接收点云后显示 XY 俯视投影</p></div><p class="help card-pad">这是雷达扫描投影，不替代用于导航的正式栅格地图。</p></section></div>
   </div>
  </div>
  <details class="card map-file-tools" id="map-file-tools"><summary>地图包导入、电脑文件与详细定位工具</summary><div class="map-file-tools-content">${files}${grid}<div class="button-row">${btn('地图文件详情','map-details','small')}${btn('本机 drmap 帮助','map-vendor-help','small')}</div></div></details>
  <details class="card mapping-log-fold"><summary>查看建图日志</summary><div id="mapping-progress" class="card-pad"></div></details>`;
};
ui['map-files-open']=()=>{$('#map-file-tools').open=true;$('#map-file-tools').scrollIntoView({behavior:'smooth'});};
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
function compactDrive(){return `<div class="mapping-dpad compact-dpad"><button data-velocity="0,0,1" aria-label="左转">↶</button><button data-velocity="1,0,0" aria-label="前进">↑</button><button data-velocity="0,0,-1" aria-label="右转">↷</button><button data-velocity="0,1,0" aria-label="左移">←</button><button class="center" data-action="stop">停止</button><button data-velocity="0,-1,0" aria-label="右移">→</button><span></span><button data-velocity="-1,0,0" aria-label="后退">↓</button><span></span></div>`;}
function routeEditorPage(){
 const single=points.length<=1;
 const runButton=single?btn('执行导航','go-goal','primary','play'):btn('执行巡检','start-nav','primary','play');
 return heading(btn('任务中心','goto-tasks','','tasks'))+operationPanel()+
 `<div class="route-workspace">
   <div class="route-map-column">
    <p class="nav-flow-hint">在地图上点选目标：只选一个点可以直接导航，继续点选会按顺序组成巡检路线。</p>
    <div class="route-action-bar" id="nav-action-bar">${btn('撤销一点','undo-waypoint','small ghost')}${btn('保存路线','save-route','','file')}${runButton}${btn('暂停','pause-nav')}${btn('继续','resume-nav')}${btn('结束任务','cancel-nav','danger')}</div>
    ${card('地图选点，自动规划路径',mapStage(),'maps','<span class="pill blue" id="nav-status">待命</span>')}
    <div id="nav-feedback" class="card nav-feedback-empty"></div>
    <details class="navigation-drive"><summary>临时手动驾驶与前视画面</summary><div class="navigation-drive-body"><div><p class="help">暂停任务后切换手动。返回路线时点击继续。</p>${btn('切换手动驾驶','nav-takeover')}${compactDrive()}<input id="motion-speed" aria-label="手动驾驶幅度" type="range" min="0.05" max="0.25" step="0.05" value="0.1"><small id="motion-readiness"></small></div><div class="camera-feed"><img id="camera-front" alt="前置相机" hidden><div id="placeholder-front" class="camera-placeholder">${icon('vision')}<strong>${S.mode==='demo'?'演示模式没有实机画面':'正在连接前置相机'}</strong></div><div class="camera-hud"><span>前视</span><span id="cam-front-tag">等待画面</span></div></div></div></details>
   </div>
   <aside class="route-editor card card-pad">
    ${mapSelect().replace('当前编辑地图','导航地图')}
    <div class="divider"></div>
    <h3 id="nav-target-title">导航目标</h3>
    <div id="goal-position" class="goal-position">尚未选择目标</div>
    <label class="field">到达朝向<select id="goal-yaw-mode"><option value="auto">沿路径末段方向</option><option value="manual">指定朝向</option></select></label>
    <div id="goal-yaw-field" hidden>${field('到达朝向 / 度','goal-yaw',0,'number')}</div>
    <div id="goal-plan-status" class="goal-plan-status">${single?'在地图上选定目标，自动规划可通行路径':'多个航点将按顺序执行；点击“执行巡检”开始。'}</div>
    <div class="divider"></div>
    ${field('路线名称','route-name',routeId?(S.routes.find(r=>r.id===routeId)?.name||'新路线'):'新路线')}
    <div class="range-label"><strong>行进顺序</strong><b id="point-count">${points.length} 个点</b></div>
    <div id="waypoints" class="waypoints"></div>
    <p class="help">航点可展开调整坐标、朝向与任务类型。</p>
    <div class="route-advanced"><h4 class="field-heading">速度、步态与方向</h4><div class="form-grid">${select('导航方式','nav-mode',[[1,'自主导航'],[0,'分段直线']])}${select('导航速度','nav-speed',[[1,'低速'],[0,'正常'],[2,'高速']])}${select('路径步态','nav-gait',[['flat','敏捷平地'],['stair','敏捷楼梯']])}${select('运动朝向','nav-manner',[[0,'前进'],[1,'倒退']])}</div></div>
    <div class="divider"></div>
    <h3>已保存路线</h3>
    <div id="route-list"></div>
    <div class="route-map-tools"><h4 class="field-heading">地图与建图工具</h4><div class="button-row">${btn('停稳并建图','start-map-here','small','maps')}${btn('原图续建 / 优化','extend-map','small','maps')}</div></div>
   </aside>
  </div>`;
}
function refreshNavActions(){
 const bar=$('#nav-action-bar');if(!bar)return;
 const single=points.length<=1;
 const run=single?btn('执行导航','go-goal','primary','play'):btn('执行巡检','start-nav','primary','play');
 bar.innerHTML=btn('撤销一点','undo-waypoint','small ghost')+btn('保存路线','save-route','','file')+run+btn('暂停','pause-nav')+btn('继续','resume-nav')+btn('结束任务','cancel-nav','danger');
 const title=$('#nav-target-title');if(title)title.textContent=single?'导航目标':'巡检航点';
 const status=$('#goal-plan-status');if(status&&!single)status.textContent='多个航点将按顺序执行；点击“执行巡检”开始。';
}
navPage=function(){
 mapTool='waypoint';
 const holder=document.createElement('div');holder.innerHTML=routeEditorPage();
 return holder.innerHTML;
};
ui['nav-takeover']=async()=>{await job('/api/nav/manual',{},'暂停路线并切换手动');await fetchState();toast('已切换手动，按住方向键移动');};
ui['undo-waypoint']=()=>{points.pop();navIntent=points.length>1?'route':'goal';renderPoints();};
function routeOptions(){return {nav_mode:+($('#nav-mode')?.value??1),speed:+($('#nav-speed')?.value??1),gait:$('#nav-gait')?.value||'flat',manner:+($('#nav-manner')?.value??0),pause_at_point:$('#route-arrival')?.value==='pause'};}
saveRoute=async function(){
 const r=await api('/api/route/save',{id:routeId,name:$('#route-name')?.value||'新巡检路线',map_id:mapId,points,options:routeOptions()});routeId=r.route.id;await fetchState();return r.route;
};
runRoute=async function(id){
 const r=S.routes.find(r=>r.id===id),options=$('#nav-mode')&&r?.id===routeId?routeOptions():r?.options||{nav_mode:1,speed:1,gait:'flat',manner:0,pause_at_point:false};
 await api('/api/nav/start',{id,...options});await fetchState();toast(S.mode==='demo'?'演示路线已开始':'路线已提交，等待导航反馈');
};
const classicRenderPoints=renderPoints;
renderPoints=function(){
 classicRenderPoints();text('goal-position',points.length?'目标：X '+fmt(points[0].x,2)+' m · Y '+fmt(points[0].y,2)+' m':'尚未选择目标');
 for(const row of $$('#waypoints .waypoint')){
  const index=+row.querySelector('[data-point]')?.dataset.point;if(!Number.isFinite(index))continue;
  const details=document.createElement('details');details.className='point-parameters';details.innerHTML='<summary>位置、朝向与类型</summary>';
  for(const child of [...row.children])if(child.matches('input,select'))details.append(child);
  const summary=document.createElement('span');summary.className='point-position';summary.textContent='航点 '+(index+1)+' · X '+fmt(points[index].x,2)+' / Y '+fmt(points[index].y,2);
  row.querySelector('button')?.before(summary);row.append(details);row.classList.add('waypoint-simple');
 }
};

const dayRender=render,dayUpdate=update;
const detailedTaskSummary=taskSummary;
taskSummary=function(){
 const nav=S?.nav,display=typeof navigationPresentation==='function'?navigationPresentation(nav):null;
 if(display)return `<div class="task-summary"><strong>${esc(display.title)}</strong><p>${esc(display.detail)}</p>${btn('查看执行诊断','native-nav-diagnostics','small','code')}</div>`;
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
  for(const [id,key] of [['nav-mode','nav_mode'],['nav-speed','speed'],['nav-gait','gait'],['nav-manner','manner']])if($('#'+id))$('#'+id).value=String(options[key]);
  if($('#route-arrival'))$('#route-arrival').value=options.pause_at_point?'pause':'continue';
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
 text('motion-readiness',ready&&S.mode!=='demo'?reason+' · '+({4097:'标准基础',4098:'兼容高台',4099:'标准楼梯',12290:'敏捷平地',12291:'敏捷楼梯'}[S.basic.Gait]||'步态待反馈')+(S.basic.ControlUsageMode===2?' · 避障'+(['未启动','空闲','未触发','正在介入'][S.basic.OOA]||'状态未知'):''):reason);
 for(const button of $$('[data-velocity]'))button.disabled=!ready;
 if(S?.nav.stop_reason==='route_complete'&&S.nav.status==='cancelling')text('nav-status','终点已到达 · 正在停止');
 if(S?.nav.stop_reason==='waypoint_pause'&&S.nav.status==='paused')text('nav-status','任务点已到达 · 等待继续');
 const navDisplay=typeof navigationFeedback==='function'?navigationFeedback(S?.nav):null;if(navDisplay){text('nav-status',navDisplay.label);html('nav-feedback',navDisplay.html);}
 const pause=$('[data-ui="pause-nav"]'),resume=$('[data-ui="resume-nav"]');if(pause)pause.disabled=S?.nav.status!=='running';if(resume)resume.disabled=S?.nav.status!=='paused';
};
document.addEventListener('click',async event=>{
 const setup=event.target.closest('[data-quick-setup]');if(setup){$('#connection-advanced').open=true;setupWizard(setup.dataset.quickSetup).catch(error=>toast(error.message,true));}
 const view=event.target.closest('[data-scan-view]');if(view){$('#mapping-views').className='mapping-views show-'+view.dataset.scanView;$$('[data-scan-view]').forEach(b=>b.classList.toggle('active',b===view));scanSequence=null;drawScan();}
});

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
  modal('平台寻路执行诊断',`<p>${esc(n.message||'等待轨迹')}</p><p>距目标 ${esc(n.distance_remaining??'—')} 米 · 重规划 ${n.replans||0} 次<br>可用轨迹 ${c.valid_candidates??'—'} / ${c.candidate_count??'—'} · 计算 ${c.compute_ms??'—'} ms</p><p class="help">归一化指令不是实际速度，预测增益不是实机标定。${S.mode==='demo'?'当前为本地演示。':''}</p><details><summary>完整诊断字段</summary><pre>${esc(JSON.stringify({controller:c,progress:n.progress_detail,plan:n.plan?.diagnostics},null,2))}</pre></details>`);return;
 }
 const r=await api('/api/nos/navigation-diagnostics',undefined,true);const n=r.current;modal('导航执行诊断',`<p>${esc(r.message||'等待诊断')}</p>${n?`<p>原厂模式：${esc(n.mode||'—')}<br>全局路径：${n.path_points} 点<br>日志输出：前进 ${fmt(n.command.forward,3)} / 侧移 ${fmt(n.command.side,3)} / 转向 ${fmt(n.command.yaw,3)}<br>原厂报告的停障距离：${fmt(n.reported_stop_distance_m,2)} m</p><p class="help">${esc(r.boundary||'')} ${r.fresh===false?'日志当前未更新，不能当成实时状态。':''}</p>`:''}<details><summary>完整诊断字段</summary><pre>${esc(JSON.stringify(r,null,2))}</pre></details>`);
};
