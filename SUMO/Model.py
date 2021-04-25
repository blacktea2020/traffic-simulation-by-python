import numpy as np
class Cf:
    def __init__(self,df):
        df = df.dropna()
        self.df = df
        self.speed = df['speed'].values
        self.acc = df['acc'].values
        self.headway = df['headway'].values
        self.spacing = df.headway.values - df.length.values
        self.delta_v = df.delta_v.values
        self.position = df.position.values
    
    def IDM(self,alpha,beta,v0,s0,T):
        s_star = s0 + T*self.speed-self.speed*self.delta_v/(2*(alpha*beta)**0.5)
        acc = alpha*(1-(self.speed/v0)**4-(s_star/self.spacing)**2)
        return acc
    
    def OV(self,alpha,L,v_max):
        OVF = 0.5*v_max*(np.tanh((self.headway-L).astype('float'))+np.tanh(L))
        acc = alpha*(OVF-self.speed)
        return acc
    
    def FVD(self,alpha,kappa,L,v_max):
        OVF = 0.5*v_max*(np.tanh((self.headway-L).astype('float'))+np.tanh(L))
        acc = alpha*(OVF-self.speed)+kappa*self.delta_v
        return acc
        