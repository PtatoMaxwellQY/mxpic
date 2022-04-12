from cmath import pi
import nazca as nd
import numpy as np

def _my_polygon (layer_wg,vtx) : 
    ''' establishing a polygon with input vertices
    Args
        vtx (2*x list) :
    Return
        frame (nazca.cell):
    '''
    sz_l = vtx.shape
    sz_l = sz_l[0]

    idx_seq = np.linspace(1,sz_l-1,sz_l-1)

    _points_ = [(vtx[0,0],vtx[0,1])]
    for idx in idx_seq:
        _point_cur_ = [(vtx[int(idx),0],vtx[int(idx),1])]
        _points_.extend(_point_cur_)
        
    frame = nd.Polygon(layer=layer_wg, points = _points_)
    
    return frame      





class circle : 
    '''
    # =================================================================
    # @ File : <mx_frame_lib.py>
    # @ structure: circle ring or disk
    # @ Args : * radius      : center radius of the ring
    #              : * width       : width of the ring 
    #              : * theta_start : start end of the ring, range [0~360], can be negative
    #              : * theta_stop  : stop  end of the ring, range [0~360], can be negative
    #              : * n_points    : resolution of the polygon
    #              : * xs          : placing layer
    # @ located in the center of the ring
    # =================================================================
    '''
    def __init__(self,radius = 10, width = 0.45, theta_start=0, theta_stop=360, n_points = 1024,xs='strip',layer=None):
        with nd.Cell(instantiate=False) as C:

            if (layer==None):
                for layers,growx,growy,acc in nd.layeriter(xs=xs):
                    (a1,b1), (a2,b2),c1,c2 = growx
                    theta = np.linspace(theta_start,theta_stop,n_points)
                    theta = theta/180*np.pi
                    vtx_outer_x = np.cos(theta)*(radius+width*a1 + b1)
                    vtx_outer_y = np.sin(theta)*(radius+width*a1 + b1)
                    vtx_inner_x = np.cos(theta)*(radius+width*a2 + b2)
                    vtx_inner_y = np.sin(theta)*(radius+width*a2 + b2)
                    vtx_outer = np.c_[vtx_outer_x,vtx_outer_y]
                    vtx_inner = np.c_[np.flip(vtx_inner_x),np.flip(vtx_inner_y)]
                    vtx = np.r_[vtx_outer,vtx_inner]
                    _my_polygon(layer_wg=layers,vtx=vtx).put(0,0,0)
                nd.Pin(name='a1',width=width,xs=xs).put(radius*np.cos(theta_start/180*np.pi),radius*np.sin(theta_start/180*np.pi),theta_start-90)
                nd.Pin(name='b1',width=width,xs=xs).put(radius*np.cos(theta_stop/180*np.pi),radius*np.sin(theta_stop/180*np.pi),theta_stop+90)            

            else:
                theta = np.linspace(theta_start,theta_stop,n_points)
                theta = theta/180*np.pi
                vtx_outer_x = np.cos(theta)*(radius+width/2)
                vtx_outer_y = np.sin(theta)*(radius+width/2)
                vtx_inner_x = np.cos(theta)*(radius-width/2)
                vtx_inner_y = np.sin(theta)*(radius-width/2)
                vtx_outer = np.c_[vtx_outer_x,vtx_outer_y]
                vtx_inner = np.c_[np.flip(vtx_inner_x),np.flip(vtx_inner_y)]
                vtx = np.r_[vtx_outer,vtx_inner]
                _my_polygon(layer_wg=layer,vtx=vtx).put(0,0,0)
                nd.Pin(name='a1',width=width,layer=layer).put(radius*np.cos(theta_start/180*np.pi),radius*np.sin(theta_start/180*np.pi),theta_start-90)
                nd.Pin(name='b1',width=width,layer=layer).put(radius*np.cos(theta_stop/180*np.pi),radius*np.sin(theta_stop/180*np.pi),theta_stop+90)            
            self.cell = C


def _my_poly_spiral(self,r,theta,order,dL,R_max):
        ''' generating a poly spiral curve
        Args
            r (2*1 list) :r[0] is the begining 
            theta (2*1 list) :theta[0] is the begining [in degree]
        Return
            frame (nazca.cell):
        '''
        theta[0] = theta[0]/180*np.pi ## angle format changing
        theta[1] = theta[1]/180*np.pi ## angle format changing

        K_ends = np.array([1/r[0],1/r[1]]) ## definition of the curvature, r[0] is the beginnin and r[1] is the ending
        L0 = np.abs(theta[0]-theta[1])/(K_ends[0] + (K_ends[1]-K_ends[0])*order/(order+1))
        L = np.linspace(0,L0,int(np.floor(L0/dL)+1)) ## L = [0:dL:L0];
        K = K_ends[0] + (K_ends[1] - K_ends[0])/np.power(L0,order)*(np.power(L0,order) - np.power(np.abs(L-L0),order))
        R = 1/K
        
        dir = np.sign(theta[1] - theta[0])
        dt = dir*dL/R

        theta_temp = np.cumsum(dt) + theta[0]

        x = np.zeros(len(L))
        y = np.zeros(len(L))
        
        idx = np.linspace(1,len(L)-1,len(L)-2+1)
        for _idx_ in idx :
            _idx_ = int(_idx_)
            x[_idx_] = x[_idx_-1] + dir*R[_idx_]*( np.sin(theta_temp[_idx_]) - np.sin(theta_temp[_idx_-1]))
            y[_idx_] = y[_idx_-1] - dir*R[_idx_]*( np.cos(theta_temp[_idx_]) - np.cos(theta_temp[_idx_-1]))

        x_end = x[len(x)-1]
        y_end = y[len(x)-1]
        x = np.r_[x[0],x[1:-2:10]]
        y = np.r_[y[0],y[1:-2:10]]

        x = np.r_[x,x_end]
        y = np.r_[y,y_end]


        vtx = np.c_[x,y]
        return vtx ## vtx = [vtx_x,vtx_y]

class spiral_single: 
    def __init__(self,R0=10,R1=10,w0=0.45,w1=0.45,angle=90,width_type='linear',spiral_order=1,dL=0.001,Rmax=100,xs='strip',layer=None) :

        ''' initiate a spiral waveguide bending with input parameters
            This is used for single type of spirals with only one trend of radius varing
        Args:
            - R0 (double) :
            - R1 (double) :
            - w0 (double) :
            - w1 (double) :
            - angle (double) :
            - width_type (str) :
            - sprial_order (double) :
            - dL (double) : minimum resolution of the sprial
            - Rmax : upper limit of the spiral when curving
            - xs (str): xsection of the vaceguide
            - layer (str): with higher priority to xs, defines the layer of waveguide
        
        '''
        vtx_line = _my_poly_spiral(self,[R0,R1],[0,angle],spiral_order,dL,Rmax) 
        x = vtx_line[:,0]
        y = vtx_line[:,1]
        z = x + 1j*y

        dz = np.diff(z)
        dz = np.r_[dz,dz[-1]]

        dir_upper = -1j*np.real(dz) + np.imag(dz)
        dir_lower = -dir_upper


        _len_ = int(len(vtx_line[:,1]))
        dL = np.power((vtx_line[1:_len_,1] - vtx_line[0:_len_-1,1]),2) + np.power((vtx_line[1:_len_,0] - vtx_line[0:_len_-1,0]),2)
        dL = np.sqrt(dL)
        L = np.cumsum(dL) ## L for each pieces
        L = np.r_[[0],L]

        L0 = sum(dL)

        if (width_type=='linear'):
            w = (w1-w0)/L0*L + w0
        elif (width_type=='dual_linear'):
            w = (w1-w0)/L0/2*np.abs(L-L0/2) + w0
        else :
            w = (w1-w0)/L0*L + w0

        with nd.Cell(instantiate=False) as C:

            if (layer==None): ## if definition is in layers
                for layers,growx,growy,acc in nd.layeriter(xs=xs):
                    (a1,b1), (a2,b2),c1,c2 = growx

                    w_xs_cur = w*a1+b1 - w*a2-b2

                    p_upper = z + dir_upper*w_xs_cur/2/abs(dir_upper)
                    p_lower = z + dir_lower*w_xs_cur/2/abs(dir_lower)

                    vtx_upper = np.c_[np.real(p_upper),np.imag(p_upper)]
                    vtx_lower = np.c_[np.real(p_lower),np.imag(p_lower)]

                    vtx = np.r_[vtx_upper,np.flip(vtx_lower,0)]

                    _my_polygon(layers,vtx).put(0,0,0)
                nd.Pin(name='a0',width=w[0],xs=xs).put(x[0],y[0],180)
                nd.Pin(name='b0',width=w[-1],xs=xs).put(x[-1],y[-1],(angle))
            else : ## if definition is in xsections
                w_xs_cur = w

                p_upper = z + dir_upper*w_xs_cur/2/abs(dir_upper)
                p_lower = z + dir_lower*w_xs_cur/2/abs(dir_lower)

                vtx_upper = np.c_[np.real(p_upper),np.imag(p_upper)]
                vtx_lower = np.c_[np.real(p_lower),np.imag(p_lower)]

                vtx = np.r_[vtx_upper,np.flip(vtx_lower,0)]
                _my_polygon(layer,vtx).put(0,0,0)
                
                nd.Pin(name='a0',width=w[0],layer=layer).put(x[0],y[0],180)
                nd.Pin(name='b0',width=w[-1],layer=layer).put(x[-1],y[-1],angle)

        self.cell =C


class spiral_dual:
    def __init__(self,R0=10,R1=10,R2=10,w0=0.45,w1=0.45,angle1=45,angle2=45,width_type='linear',spiral_order=1,dL=0.001,Rmax=100,xs='strip',layer=None) :
        ''' initiate a spiral waveguide bending with input parameters
            This is used for single type of spirals with only one trend of radius varing
        Args:
            - R0 (double) :
            - R1 (double) :
            - R2 (double) :
            - w0 (double) :
            - w1 (double) :
            - angle (double) :
            - width_type (str) :
            - sprial_order (double) :
            - dL (double) : minimum resolution of the sprial
            - Rmax : upper limit of the spiral when curving
            - xs (str): xsection of the vaceguide
            - layer (str): with higher priority to xs, defines the layer of waveguide
        
        '''
        self.R0 = R0
        self.R1 = R1
        self.R2 = R2
        self.w0 = w0
        self.w1 = w1
        self.angle1=angle1
        self.angle2=angle2
        self.angle=angle1+angle2
        self.width_type=width_type
        self.spiral_order=spiral_order
        self.dL=dL
        self.Rmax=Rmax
        self.xs=xs
        self.layer=layer

        
        vtx_line_begin = _my_poly_spiral(self,[R0,R1],[0,angle1],spiral_order,dL,Rmax) 
        vtx_line_end = _my_poly_spiral(self,[R1,R2],[angle1,angle1+angle2],spiral_order,dL,Rmax) 
        
        x = np.r_[vtx_line_begin[0:-1,0],vtx_line_end[:,0]+vtx_line_begin[-1,0]]
        y = np.r_[vtx_line_begin[0:-1,1],vtx_line_end[:,1]+vtx_line_begin[-1,1]]
        z = x + 1j*y

        dz = np.diff(z)
        dz = np.r_[dz,dz[-1]]

        dir_upper = -1j*np.real(dz) + np.imag(dz)
        dir_lower = -dir_upper


        _len_ = int(len(y))
        dL = np.power((y[1:_len_] - y[0:_len_-1]),2) + np.power((x[1:_len_] - x[0:_len_-1]),2)
        dL = np.sqrt(dL)
        L = np.cumsum(dL) ## L for each pieces
        L = np.r_[[0],L]

        L0 = sum(dL)

        if (width_type=='linear'):
            w = (w1-w0)/L0*L + w0
        elif (width_type=='dual_linear'):
            w = (w1-w0)/L0/2*np.abs(L-L0/2) + w0
        else :
            w = (w1-w0)/L0*L + w0

        with nd.Cell(instantiate=False) as C:

            if (layer==None): ## if definition is in layers
                for layers,growx,growy,acc in nd.layeriter(xs=xs):
                    (a1,b1), (a2,b2),c1,c2 = growx

                    w_xs_cur = w*a1+b1 - w*a2-b2

                    p_upper = z + dir_upper*w_xs_cur/2/abs(dir_upper)
                    p_lower = z + dir_lower*w_xs_cur/2/abs(dir_lower)

                    vtx_upper = np.c_[np.real(p_upper),np.imag(p_upper)]
                    vtx_lower = np.c_[np.real(p_lower),np.imag(p_lower)]

                    vtx = np.r_[vtx_upper,np.flip(vtx_lower,0)]

                    _my_polygon(layers,vtx).put(0,0,0)
                nd.Pin(name='a0',width=w[0],xs=xs).put(x[0],y[0],180)
                nd.Pin(name='b0',width=w[-1],xs=xs).put(x[-1],y[-1],(angle1+angle2))
            else : ## if definition is in xsections
                w_xs_cur = w

                p_upper = z + dir_upper*w_xs_cur/2/abs(dir_upper)
                p_lower = z + dir_lower*w_xs_cur/2/abs(dir_lower)

                vtx_upper = np.c_[np.real(p_upper),np.imag(p_upper)]
                vtx_lower = np.c_[np.real(p_lower),np.imag(p_lower)]

                vtx = np.r_[vtx_upper,np.flip(vtx_lower,0)]
                _my_polygon(layer,vtx).put(0,0,0)
                
                nd.Pin(name='a0',width=w[0],layer=layer).put(x[0],y[0],180)
                nd.Pin(name='b0',width=w[-1],layer=layer).put(x[-1],y[-1],angle1+angle2)

        self.sz = [np.abs(x[-1] - x[0]),np.abs(y[-1] - x[1])]
        self.cell =C
    
    def racetrack(self,dLx=0,dLy=0):
        with nd.Cell(instantiate=False) as C:
            if (self.layer==None):
                if (self.angle==90):
                    nd.strt(length=dLx,width=self.w0,xs=self.xs).put(-dLx/2,-dLy/2-self.sz[1],0)
                    temp = self.cell.put()
                    temp = nd.strt(length=dLy,width=self.w1,xs=self.xs).put()
                    temp = self.cell.put('b0',temp.pin['b0'].xya(),flip=1)
                    temp = nd.strt(length=dLx,width=self.w0,xs=self.xs).put(temp.pin['a0'].xya())
                    temp = self.cell.put()
                    temp = nd.strt(length=dLy,width=self.w1,xs=self.xs).put()
                    temp = self.cell.put('b0',temp.pin['b0'].xya(),flip=1)
                elif (self.angle==180): ## in this case, dy is not used
                    temp = nd.strt(length=dLx,width=self.w0,xs=self.xs).put(-dLx/2,-dLy/2-self.sz[1],0)
                    sp_r = self.cell.put()
                    sp_l = self.cell.put(temp.pin['a0'].xya(),flip=1)
                    temp = nd.strt(length=np.abs(sp_r.pin['b0'].x-sp_l.pin['b0'].x),width=sp_r.pin['b0'].width,xs=self.xs).put()
            else:
                if (self.angle==90):
                    nd.strt(length=dLx,width=self.w0,layer=self.layer).put(-dLx/2,-dLy/2-self.sz[1],0)
                    temp = self.cell.put()
                    temp = nd.strt(length=dLy,width=self.w1,layer=self.layer).put()
                    temp = self.cell.put('b0',temp.pin['b0'].xya(),flip=1)
                    temp = nd.strt(length=dLx,width=self.w0,layer=self.layer).put(temp.pin['a0'].xya())
                    temp = self.cell.put()
                    temp = nd.strt(length=dLy,width=self.w1,layer=self.layer).put()
                    temp = self.cell.put('b0',temp.pin['b0'].xya(),flip=1)
                elif (self.angle==180): ## in this case, dy is not used
                    temp = nd.strt(length=dLx,width=self.w0,layer=self.layer).put(-dLx/2,-dLy/2-self.sz[1],0)
                    sp_r = self.cell.put()
                    sp_l = self.cell.put(temp.pin['a0'].xya(),flip=1)
                    temp = nd.strt(length=np.abs(sp_r.pin['b0'].x-sp_l.pin['b0'].x),width=sp_r.pin['b0'].width,layer=self.layer).put()

            self.rck_sz = [2*self.sz[0]+dLx,2*self.sz[1]+dLy]

        return C
