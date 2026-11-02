'use strict';
let shareState=null;
function renderSharing(){
 $('#sharing-control')?.remove();
 if(!shareState?.enabled)return;
 const container=$('#connection-content');if(!container)return;
 let panel=$('#sharing-panel');
 if(!panel){panel=document.createElement('section');panel.id='sharing-panel';panel.className='card card-pad';container.append(panel);}
 const markup='<strong>同一 Wi-Fi 直接使用</strong><p class="help">其他电脑打开以下地址即可操作，无需口令或申请控制权。</p><div class="pre">'+esc(shareState.urls.join('\n'))+'</div><p class="help">主机电脑保持运行。共享网段：'+esc(shareState.subnet)+'</p>';
 if(panel._shareMarkup!==markup){panel.innerHTML=markup;panel._shareMarkup=markup;}
}
async function refreshSharing(){try{shareState=await api('/api/sharing',undefined,true);renderSharing();}catch(e){}}
const sharingConnectionCenter=connectionCenter;
connectionCenter=function(){sharingConnectionCenter();renderSharing();};
document.addEventListener('DOMContentLoaded',refreshSharing);
setInterval(()=>{if(page==='settings')refreshSharing();},5000);
