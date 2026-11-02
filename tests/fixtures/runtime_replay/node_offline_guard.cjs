'use strict';
// Defence in depth: the acceptance runner also applies macOS deny network*.
// Do not log arguments: URLs or process arguments could contain credentials.
const blocked=[];
const deny=event=>function(){blocked.push(event);throw Error('Offline acceptance forbids '+event);};
const net=require('node:net');
net.Socket.prototype.connect=deny('net.connect');
net.Server.prototype.listen=deny('net.listen');
const datagram=require('node:dgram');
datagram.Socket.prototype.send=deny('dgram.send');
datagram.Socket.prototype.bind=deny('dgram.bind');
const dns=require('node:dns');
for(const key of Object.keys(dns))if(/^(lookup|resolve|reverse)/.test(key)&&typeof dns[key]==='function')dns[key]=deny('dns.'+key);
for(const name of ['node:http','node:https']){
 const module=require(name);module.request=deny(name+'.request');module.get=deny(name+'.get');
}
const child=require('node:child_process');
for(const key of ['exec','execFile','execSync','execFileSync','spawn','spawnSync','fork'])child[key]=deny('child_process.'+key);
globalThis.fetch=deny('fetch');
if(globalThis.WebSocket)globalThis.WebSocket=deny('WebSocket');
process.once('exit',()=>process.stderr.write('OFFLINE_NODE_GUARD='+JSON.stringify({blocked})+'\n'));
