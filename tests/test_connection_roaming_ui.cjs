const {test}=require('node:test');const assert=require('node:assert/strict');const vm=require('node:vm');const fs=require('node:fs');
const source=fs.readFileSync('platform_web/connections.js','utf8');
function setup(){
 const nodes={'#sdk-follows-aos':{checked:true},'#sdk-host':{value:'old'},'#sdk-user':{value:'old-user'},'#sdk-ssh-port':{value:'22'},'#hub-aos-ip':{value:'198.51.100.7'},'#hub-aos-ssh-user':{value:'user'},'#hub-aos-ssh-port':{value:'2222'}};
 const c=vm.createContext({$:id=>nodes[id]});vm.runInContext(source.slice(source.indexOf('function paintSDKAddressFollow(){'),source.indexOf("document.addEventListener('change'")),c);return {c,nodes};
}
test('same-host SDK fields track the single AOS endpoint',()=>{
 const {c,nodes}=setup();c.paintSDKAddressFollow();assert.equal(nodes['#sdk-host'].value,'198.51.100.7');assert.equal(nodes['#sdk-host'].disabled,true);assert.equal(nodes['#sdk-ssh-port'].value,'2222');
});
test('independent SDK fields stay editable and retain their address',()=>{
 const {c,nodes}=setup();nodes['#sdk-follows-aos'].checked=false;c.paintSDKAddressFollow();assert.equal(nodes['#sdk-host'].value,'old');assert.equal(nodes['#sdk-host'].disabled,false);
});
