"""Connection-scoped, single-use browser navigation intents.

Call under the platform state lock. A cancellation may arrive before the start
request, so revoked IDs remain in memory for the whole connection generation.
Changing the connection discards that generation; its requests still fail the
explicit server/connection identity check. No I/O or task cancellation here.
"""
import re


class NavigationIntents:
    def __init__(self):
        self.epoch=None
        self.records={}

    def _token(self,req,instance,generation):
        value=req.get('client_intent')
        if not isinstance(value,str) or not re.fullmatch(r'[A-Za-z0-9_-]{16,96}',value):
            raise ValueError('导航请求缺少有效的客户端意图编号')
        supplied=req.get('connection_generation')
        if (req.get('server_instance')!=instance or isinstance(supplied,bool) or
                not isinstance(supplied,int) or supplied!=generation):
            raise ValueError('导航请求属于旧服务或旧连接，请刷新后重新开始')
        epoch=(instance,generation)
        if self.epoch!=epoch:self.epoch=epoch;self.records={}
        return (instance,generation,value)

    def register(self,req,instance,generation):
        # Legacy callers retain their existing stop/generation boundaries.
        # The automatic preparation UI requires the advertised new protocol.
        if 'client_intent' not in req:return None
        token=self._token(req,instance,generation)
        if token[2] in self.records:raise ValueError('本次导航意图已使用或已撤销，请重新点击开始')
        self.records[token[2]]='registered'
        return token

    def revoke(self,req,instance,generation):
        token=self._token(req,instance,generation)
        self.records[token[2]]='revoked'
        return token

    def valid(self,token,instance,generation):
        return token is None or (token[:2]==(instance,generation) and
            self.epoch==token[:2] and self.records.get(token[2])=='registered')

    def check(self,token,instance,generation):
        if not self.valid(token,instance,generation):raise ValueError('本次导航意图已撤销或连接已变化')

    @staticmethod
    def nav_fields(token):
        return {} if token is None else dict(client_intent=token[2],client_intent_context=list(token[:2]))

    @staticmethod
    def owns(nav,token):
        return nav.get('client_intent')==token[2] and nav.get('client_intent_context')==list(token[:2])
