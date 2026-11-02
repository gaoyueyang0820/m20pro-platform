'use strict';
// Presentation only: a running task or accepted command is not proof of motion.
function navigationPresentation(nav){
 if(!nav)return null;
 const phase=nav.phase||'',state=nav.execution_state||'';
 if(phase==='validating'||phase==='preparing')return {label:'准备导航',title:'正在确认导航就绪',detail:nav.message||'正在核对地图、控制模式和步态；尚未下发目标',severity:'normal'};
 if(state==='stop_unconfirmed'||phase==='cancel_unconfirmed'||phase==='cancel_rejected')return {label:'停止未确认',title:'停止状态尚未确认',detail:nav.cancel_error||nav.error||'请核对连接与机器人状态；确认前不会开始另一项任务',severity:'error'};
 if(nav.status==='cancelling')return {label:'正在确认停止',title:nav.stop_reason==='route_complete'?'目标已到达，正在确认停车':'正在确认停止导航',detail:nav.warning||(nav.engine==='platform'?'零速度已发送，等待新鲜的实际停稳反馈':'已发送停止请求，等待对应回包'),severity:'normal'};
 if(state==='waiting_progress')return {label:'等待实际进展',title:'导航暂时没有继续前进',detail:nav.warning||nav.message||'任务仍在等待通路，平台正在核对实际位移',severity:'waiting'};
 if(state==='awaiting_progress'||state==='starting')return {label:'等待实际进展',title:'目标已下发，等待机器人行动',detail:nav.message||'以位姿和任务反馈确认进展',severity:'normal'};
 if(nav.status==='paused'&&nav.stop_reason==='no_progress')return {label:'已暂停 · 无进展',title:'持续没有进展，已确认停止',detail:nav.warning||nav.message||'请核对现场通路后继续或重新规划',severity:'waiting'};
 if(nav.status==='failed')return {label:'任务未完成',title:phase==='validation_failed'||phase==='validation_timeout'||phase==='preparation_failed'?'导航尚未开始':'导航未完成',detail:nav.error||nav.message||'请查看具体反馈',severity:'error'};
 if(nav.status==='completed')return {label:'已完成',title:'全部目标已完成',detail:'导航已结束，停止状态已确认',severity:'normal'};
 if(nav.status==='cancelled')return {label:'已取消',title:'任务已取消',detail:'没有继续执行的导航任务',severity:'normal'};
 if(nav.engine==='platform'&&nav.status==='running')return {label:'平台自主寻路',title:'正在根据实时扫描跟踪通路',detail:`${nav.message||'正在跟踪通路'} · 距目标 ${nav.distance_remaining??'—'} 米 · 已重规划 ${nav.replans||0} 次`,severity:'normal'};
 return null;
}
function previousStopPresentation(notice){
 if(!notice||!['unconfirmed','active_after_reconnect'].includes(notice.state))return null;
 return {title:'旧连接的停止状态尚未确认',detail:notice.message||'请核对旧设备或使用手柄停止；当前离线页面不能证明旧设备已经停稳'};
}
function navigationFeedback(nav){
 const view=navigationPresentation(nav);if(!view)return null;
 const escape=v=>String(v??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
 return {label:view.label,html:`<div class="kv"><span>当前状态</span><b>${escape(view.label)}</b></div><div class="kv"><span>完成航点</span><b>${Number(nav.completed)||0} / ${nav.route?.points?.length||0}</b></div><p class="help nav-progress-warning ${view.severity==='error'?'red':''}">${escape(view.detail)}</p><button type="button" class="btn small" data-ui="native-nav-diagnostics">查看执行诊断</button>`};
}
if(typeof module!=='undefined'&&module.exports)module.exports={navigationPresentation,previousStopPresentation,navigationFeedback};
