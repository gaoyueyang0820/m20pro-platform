'use strict';

const iotNames=new Map();
const iotScope=()=>[S?.server_instance,S?.connection_generation,S?.mode].join('|');
const iotName=id=>iotNames.get(String(id))||'设备 '+id;
const iotSummary=ids=>ids.length?ids.map(iotName).join('、'):'尚未选择设备';
const iotSelected=input=>JSON.parse(input.value);

function iotPicker(ids, attributes, label='设备'){
 return `<div class="field iot-picker" data-iot-picker><span>${esc(label)}</span>
  <input type="hidden" data-iot-selection ${attributes} value="${esc(JSON.stringify(ids))}">
  <small data-iot-picked>${esc(iotSummary(ids))}</small>
  <button type="button" class="btn small" data-iot-pick>选择设备</button></div>`;
}

function iotReadings(rows){
 return rows.map(row=>{
  const values=Object.entries(row.measurements||{}).map(([name,value])=>esc(name)+' '+esc(value)).join(' · ');
  return `<small>${esc(iotName(row.device_id))} · ${values||esc(row.error||'暂无读数')}</small>`;
 }).join('');
}

function openIoTDevices(picker=null){
 const input=picker?.querySelector('[data-iot-selection]'),selection=new Set(input?iotSelected(input):[]);
 const scope=iotScope();
 modal(picker?'选择物联网设备':'物联网设备',`<div id="iot-device-browser" class="stack">
  <div class="form-grid">${field('设备名称','iot-device-search','')}
   <div class="button-row"><button type="button" class="btn small" data-iot-search>查询 / 刷新</button>${btn('连接设置','iot-settings','small')}</div></div>
  <div data-iot-list class="iot-device-list"></div>
  <p class="help" data-iot-device-status role="status"></p>
  <div class="button-row"><button type="button" class="btn small" data-iot-page="-1">上一页</button>
   <button type="button" class="btn small" data-iot-page="1">下一页</button>
   ${picker?'<button type="button" class="btn small primary" data-iot-apply>应用选择</button>':''}</div>
  <div data-iot-readout class="iot-device-list"></div></div>`);
 const root=$('#iot-device-browser'),status=root.querySelector('[data-iot-device-status]');
 const active=()=>root.isConnected&&$('#modal')?.open&&iotScope()===scope;
 let page=1,generation=0,reading=0;
 function showSelection(){if(picker)status.textContent=`已选 ${selection.size} 台设备`;}
 async function load(next=1){
  const turn=++generation;
  status.textContent='正在读取设备清单…';
  root.querySelectorAll('[data-iot-page]').forEach(button=>button.disabled=true);
  try{
   const name=root.querySelector('#iot-device-search').value.trim();
   const result=await api('/api/iot/devices?'+new URLSearchParams({page:next,name}),undefined,true);
   if(!active()||turn!==generation)return;
   page=result.page;
   for(const item of result.devices)iotNames.set(item.id,item.name);
   const states={online:'在线',offline:'离线',unknown:'状态未知'};
   root.querySelector('[data-iot-list]').innerHTML=result.devices.map(item=>picker
    ? `<label class="check iot-device-choice"><input type="checkbox" data-iot-device="${esc(item.id)}" ${selection.has(item.id)?'checked':''}><span>${esc(item.name)} <small>${states[item.status]}</small></span></label>`
    : `<div class="route-row"><span>${esc(item.name)} <small>${states[item.status]}</small></span><button type="button" class="btn small" data-iot-read="${esc(item.id)}">读数</button></div>`).join('');
   root.querySelector('[data-iot-page="-1"]').disabled=page<=1;
   root.querySelector('[data-iot-page="1"]').disabled=!result.has_more;
   status.textContent=result.devices.length?`第 ${page} 页${picker?' · 已选 '+selection.size+' 台设备':''}`:'没有符合条件的设备';
  }catch(error){if(active()&&turn===generation)status.textContent=error.message;}
 }
 root.addEventListener('change',event=>{
  const id=event.target.dataset.iotDevice;if(!id)return;
  if(event.target.checked&&selection.size>=8){event.target.checked=false;status.textContent='每项任务最多选择 8 台设备';return;}
  if(event.target.checked)selection.add(id);else selection.delete(id);
  showSelection();
 });
 root.addEventListener('keydown',event=>{
  if(event.target.id==='iot-device-search'&&event.key==='Enter'){event.preventDefault();load();}
 });
 root.addEventListener('click',async event=>{
  const button=event.target.closest('button');if(!button||!active())return;
  if(button.hasAttribute('data-iot-search'))load();
  if(button.dataset.iotPage)load(page+Number(button.dataset.iotPage));
  if(button.hasAttribute('data-iot-apply')&&input?.isConnected){
   const ids=[...selection];input.value=JSON.stringify(ids);
   picker.querySelector('[data-iot-picked]').textContent=iotSummary(ids);
   input.dispatchEvent(new Event('change',{bubbles:true}));$('#modal').close();
  }
  const id=button.dataset.iotRead;if(!id)return;
  const turn=++reading,out=root.querySelector('[data-iot-readout]');
  out.textContent='正在读取 '+iotName(id)+'…';
  try{
   const row=await api('/api/iot/read?device_id='+encodeURIComponent(id),undefined,true);
   if(active()&&turn===reading)out.innerHTML=iotReadings([row]);
  }catch(error){if(active()&&turn===reading)out.textContent=error.message;}
 });
 load();
}

ui['iot-devices']=()=>openIoTDevices();
ui['iot-settings']=async()=>{
 const scope=iotScope();
 if(S?.mode!=='live'){modal('物联网连接','<p class="help">连接 GOS 后可验证并保存物联网平台。</p>');return;}
 const saved=await api('/api/iot/settings');if(iotScope()!==scope)return;
 modal('物联网连接',`<form id="iot-connection-form">
  ${field('物联网 API 地址','iot-url',saved.url,'url')}
  <div class="form-grid">${field('账号','iot-user',saved.user)}${field('密码','iot-password','','password')}</div>
  ${field('授权令牌（也可用账号登录）','iot-token','','password')}
  <div data-iot-challenge hidden><img alt="物联网登录验证码" data-iot-captcha>
   ${field('验证码','iot-code','')}</div>
  <p data-iot-connection-status class="help" role="status">${saved.configured?'已配置，原凭据可留空沿用。':'由 GOS 访问物联网平台，验证通过后保存。'}</p>
  <button type="submit" class="btn primary">验证并保存</button></form>`);
 const form=$('#iot-connection-form'),status=form.querySelector('[data-iot-connection-status]');
 let challenge=null;
 form.querySelectorAll('input[type=password]').forEach(input=>input.autocomplete='new-password');
 form.addEventListener('submit',async event=>{
  event.preventDefault();
  const button=form.querySelector('button[type=submit]');
  if(button.disabled||iotScope()!==scope)return;
  const value=id=>form.querySelector('#iot-'+id).value;
  const url=value('url').trim(),user=value('user').trim();
  const params={url,user,password:value('password'),token:value('token')};
  if(challenge&&challenge.url===url&&challenge.user===user){params.uuid=challenge.uuid;params.code=value('code').trim();}
  status.textContent='正在验证设备清单…';button.disabled=true;
  try{
   const result=await api('/api/iot/settings',params,true);
   if(!form.isConnected||!$('#modal')?.open||iotScope()!==scope)return;
   if(result.challenge){
    challenge={...result.challenge,url,user};
    form.querySelector('[data-iot-challenge]').hidden=false;
    form.querySelector('[data-iot-captcha]').src='data:image/jpeg;base64,'+result.challenge.image;
    status.textContent='请填写验证码，再点击验证并保存';form.querySelector('#iot-code').focus();return;
   }
   iotNames.clear();
   form.querySelectorAll('input[type=password]').forEach(input=>input.value='');
   openIoTDevices();toast('物联网平台已连接');
  }catch(error){if(form.isConnected){status.textContent=error.message;challenge=null;form.querySelector('[data-iot-challenge]').hidden=true;}}
  finally{button.disabled=false;}
 });
};

document.addEventListener('click',event=>{
 const button=event.target.closest('[data-iot-pick]');
 if(button)openIoTDevices(button.closest('[data-iot-picker]'));
});
