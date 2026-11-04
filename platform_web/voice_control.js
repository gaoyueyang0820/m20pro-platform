'use strict';
// Each click authorizes one short listening session, never background listening.
(function(){
 const button=$('#voice-toggle'),status=$('#voice-status');if(!button)return;
 const Recognition=window.SpeechRecognition||window.webkitSpeechRecognition;
 let current=null,generation=0,pending=false;
 const scope=()=>[page,S?.mode,S?.ip].join('|');
 const show=message=>{if(status){status.textContent=message;status.hidden=!message;}};
 function stop(message=''){
  generation++;const session=current;current=null;
  if(session){clearTimeout(session.timer);session.recognition.onresult=null;session.recognition.onend=null;session.recognition.onerror=null;try{session.recognition.abort();}catch(_){}}
  button.classList.remove('voice-on');button.setAttribute('aria-pressed','false');show(message);
 }
 function valid(session){return current===session&&session.generation===generation&&Date.now()<session.expires&&document.visibilityState==='visible'&&session.scope===scope()&&!authPending;}
 async function result(session,event){
  if(!valid(session)){stop('语音已取消，请重新点击');return;}
  const finals=[];
  for(let i=event.resultIndex;i<event.results.length;i++)if(event.results[i].isFinal)finals.push(event.results[i][0].transcript);
  if(!finals.length)return;
  const phrase=finals.length===1?finals[0].trim().replace(/[。！!，,？?]+$/,''):'';
  const commands={起立:'stand',站起:'stand',站起来:'stand',站立:'stand',起身:'stand',趴下:'lie',趴下来:'lie',卧下:'lie'};
  const action=Object.hasOwn(commands,phrase)?commands[phrase]:null;
  if(!action){stop('未执行。请点击后单独说“起立”或“趴下”');return;}
  // Recheck immediately before dispatch; consume authorization before the request.
  if(!valid(session))return;
  stop('识别到“'+phrase+'”，正在发送');pending=true;button.disabled=true;
  try{await api('/api/cmd',{action});if(session.scope===scope())show('“'+phrase+'”指令已发送，请查看设备状态');}
  catch(error){show('语音指令失败：'+error.message);}
  finally{pending=false;button.disabled=false;}
 }
 button.addEventListener('click',()=>{
  if(pending)return;if(current){stop('语音已关闭');return;}
  if(!Recognition){show('当前浏览器不支持语音识别，请使用按钮');return;}
  if(authPending||!S||document.visibilityState!=='visible'){show('请等待平台状态就绪');return;}
  const recognition=new Recognition();
  const session={recognition,generation:++generation,scope:scope(),expires:Date.now()+15000};current=session;
  recognition.lang='zh-CN';recognition.continuous=false;recognition.interimResults=false;
  recognition.onresult=event=>result(session,event);
  recognition.onend=()=>{if(current===session)stop('聆听已结束，点击后可再说一次');};
  recognition.onerror=event=>{if(current===session)stop(event.error==='not-allowed'?'麦克风权限未开启，请检查浏览器设置':'语音识别未完成，请重试或使用按钮');};
  session.timer=setTimeout(()=>{if(current===session)stop('聆听超时，请重新点击');},15000);
  try{recognition.start();button.classList.add('voice-on');button.setAttribute('aria-pressed','true');show('请单独说“起立”或“趴下”（本次聆听15秒）');}
  catch(_){stop('无法启动语音识别，请检查浏览器权限');}
 });
 const cancel=()=>{if(current)stop('语音已取消');};
 window.addEventListener('blur',cancel);window.addEventListener('hashchange',cancel);window.addEventListener('pagehide',cancel);
 document.addEventListener('visibilitychange',()=>{if(document.visibilityState!=='visible')cancel();});
 document.addEventListener('click',event=>{if(event.target.closest('[data-action],[data-page],[data-velocity],[data-ui="motion-stop"]'))cancel();},true);
})();
