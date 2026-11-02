'use strict';
const modelVisionPage=vision;
let modelCheckBusy=false,modelCheckMarkup='';
function paintModelCheck(){
 if(page!=='vision')return;
 const box=$('#model-check-result');
 if(box){box.hidden=!modelCheckMarkup;if(box._markup!==modelCheckMarkup){box.innerHTML=modelCheckMarkup;box._markup=modelCheckMarkup;}}
 for(const el of $$('[data-ui^="check-"]'))el.disabled=modelCheckBusy;
}
vision=function(){
 const layout=document.createElement('div');layout.innerHTML=modelVisionPage();
 const models=S.vision_models||[];
 const cards=models.map(m=>`<section class="card card-pad pretrained-card"><div class="pretrained-top"><span class="eyebrow">${m.id==='ppe'?'HELMET & PPE':m.id==='counting'?'PEOPLE COUNTING':'PERSON DETECTION'}</span><span class="pill ${m.installed?'':'amber'}">${m.installed?'模型已就绪':'模型文件缺失'}</span></div><h2>${esc(m.id==='ppe'?'安全帽与防护用品':m.id==='counting'?'人数统计 · YOLO12n':'人员跟随模型')}</h2><p>${m.id==='ppe'?'识别安全帽佩戴情况、人员与反光背心等防护用品。':m.id==='counting'?'统计画面中的当前人数、峰值和累计人员轨迹。':'供人员跟随使用，自动检测画面中的人员。'}</p><div class="pretrained-footer">${btn('用图片检查模型','check-'+m.id,'small','play')}<a href="${esc(m.source||'#')}" target="_blank" rel="noopener">${m.id==='counting'?'模型说明':'查看来源'} ↗</a></div></section>`).join('');
 const slot=layout.querySelector('[data-vision-pane="models"]');
 if(slot)slot.innerHTML=`<div class="pretrained-models">${cards}</div>${operationPanel()}`;
 // Results belong to the whole vision workspace, including the people tab.
 layout.insertAdjacentHTML('beforeend',`<section class="card model-proof" id="model-check-result" role="status" ${modelCheckMarkup?'':'hidden'}>${modelCheckMarkup}</section>`);
 if(S.mode==='demo'){
  const start=layout.querySelector('[data-ui="ppe-start"]');if(start){start.disabled=true;start.title='连接机器人后即可开启；本机图片检查不需要机器人';}
 }
 return layout.innerHTML;
};
for(const kind of ['person','ppe','counting'])ui['check-'+kind]=async()=>{
 if(modelCheckBusy)return;modelCheckBusy=true;
 modelCheckMarkup='<div class="card-pad"><h2>正在检查模型…</h2><p class="help">正在处理样例图片…</p></div>';
 paintModelCheck();$('#model-check-result')?.scrollIntoView({block:'nearest',behavior:'smooth'});
 try{
  const result=await job('/api/vision/check',{kind},'检查预训练模型');
  modelCheckMarkup=`<div class="model-proof-title"><div><span class="eyebrow">LOCAL INFERENCE</span><h2>${kind==='ppe'?'安全帽模型':kind==='counting'?'人数统计模型':'人员模型'} · 本机检查完成</h2><p class="help">样例图片 · 非实时画面 · 处理耗时 ${fmt(result.infer_ms)} ms</p></div><span class="pill">图片识别</span></div><div class="model-proof-body"><img src="${esc(result.image)}" alt="样例图片的识别结果"><div>${Object.entries(result.counts).map(([name,count])=>kv(({Person:'人员',person:'人员',Hardhat:'戴安全帽','NO-Hardhat':'未戴安全帽','Safety Vest':'反光背心','NO-Safety Vest':'未穿反光背心','NO-Mask':'未戴口罩'})[name]||esc(name),count)).join('')||'<p>此样例未检测到目标。</p>'}<p class="help">结果受光照、遮挡和目标大小影响；可在相机画面中查看现场效果。</p></div></div>`;
  paintModelCheck();if(page==='vision')$('#model-check-result')?.scrollIntoView({block:'nearest',behavior:'smooth'});
 }catch(e){modelCheckMarkup=`<div class="card-pad"><h2>模型检查未完成</h2><p>${esc(e.message)}</p><p class="help">处理提示的问题后重试。</p></div>`;toast(e.message,true);}finally{modelCheckBusy=false;paintModelCheck();}
};
const modelVisionUpdate=update;
update=function(){modelVisionUpdate();if(page!=='vision')return;paintModelCheck();
 const on=$('[data-ui="ppe-start"]'),off=$('[data-ui="ppe-stop"]');
 if(on)on.disabled=S.mode==='demo'||!!S.ppe?.enabled;
 if(off)off.disabled=!S.ppe?.enabled;
};
ui['ppe-stop']=async()=>{const camera=S.ppe?.camera;await api('/api/ppe/stop',{});await fetchState();if(camera&&$('#camera-'+camera))cameraOpen(camera);toast('安全帽识别已停止');};
