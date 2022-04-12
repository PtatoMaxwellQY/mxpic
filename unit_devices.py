import nazca as nd
import numpy as np

from .structures import *

class Vias:
    def __init__ (self,xs_via,area,via_sz,via_spacing):
        
        with nd.Cell(instantiate=False) as C:

            via_period_x = via_sz[0] + via_spacing[0]
            num_x = int(np.floor(area[0]/via_period_x))
            num_x = np.linspace(-(num_x-1)/2,(num_x-1)/2,num_x)

            via_period_y = via_sz[1] + via_spacing[1]
            num_y = int(np.floor(area[1]/via_period_y))
            num_y = np.linspace(-(num_y-1)/2,(num_y-1)/2,num_y)

            for _idx_x_ in num_x:
                for _idx_y_ in num_y:
                    nd.strt(length=via_sz[0],width=via_sz[1],xs=xs_via).put(_idx_x_*via_period_x-via_sz[0]/2,_idx_y_*via_period_y,0)
            
        self.cell = C
        self.num_vias = num_x*num_y

class ISL: ## thermal isolation
    def __init__ (self,xs,width,length):
        with nd.Cell(instantiate=False) as C:
            for layers,growx,growy,acc in nd.layeriter(xs=xs):
                (a1,b1), (a2,b2),c1,c2 = growx
                _L_ = length*(a1-a2)+(b1-b2)
                nd.strt(length=_L_,width=width*(a1-a2)+(b1-b2),layer=layers).put(-(b1-b2)/2,0,0)

        self.cell = C

class PADs:
    def __init__(self,tapout,num,length=80,width=60,edge=5,rows=1,x_spacing=100,y_spacing=120):
        with nd.Cell(instantiate=False) as C: 
            idx = np.linspace(0,num-1,num)
            row_idx = np.linspace(0,rows-1,rows)
            for _row_ in row_idx:
                for _idx_ in idx:
                    _pad_ = nd.strt(length=length,width=width,layer=tapout.LAYER_PAD_OPEN).put(_idx_*x_spacing + _row_*x_spacing/2,-length/2 - _row_*y_spacing ,90)
                    nd.Pin(name='e'+str(int(_idx_*rows+_row_+1)),pin=_pad_.pin['b0']).put()
                    
                    if (hasattr(tapout,'BOND_PAD')==True):
                        nd.strt(length=length+2*edge,width=width+2*edge,layer=tapout.LAYER_BOND_PAD).put(_idx_*x_spacing + _row_*x_spacing/2,-length/2-edge - _row_*y_spacing,90)

                    if (hasattr(tapout,'LAYER_METAL_2')==True):
                        nd.strt(length=length+2*edge,width=width+2*edge,layer=tapout.LAYER_METAL_2).put(_idx_*x_spacing + _row_*x_spacing/2,-length/2-edge - _row_*y_spacing,90)
                    else :
                        nd.strt(length=length+2*edge,width=width+2*edge,layer=tapout.LAYER_METAL).put(_idx_*x_spacing + _row_*x_spacing/2,-length/2-edge - _row_*y_spacing,90)

        self.cell = C

'''
# =================================================================
# @ File : <mx_structure_lib.py>
# @ Device: coupler connected with Euler bends
# @ Parameters : * layer    : layer of the coupler
#              : * w_wg     : the width of the waveguide connected to the outside
#              : * w_cp     : the width of the coupler
#              : * dLc      : for straight coupler, the coupling length
#              : * dAc      : for bend coupler, the coupling angle
# =================================================================
'''
class coupler_input : 
    def __init__(self,xs='strip',attach_type='circular',w_wg=0.45,w_cp=0.45,R_att=10,R_wg=20,R_cp=5,dLc=0,dAc=0,theta_attach=60,coupler_type = 'DC') :
        with nd.Cell(instantiate=False) as C:
            if (coupler_type=='straight' or coupler_type=='s' or coupler_type=='DC'):
                cp = nd.strt(length=dLc,width=w_cp,xs=xs).put(-dLc/2,0,0)
                al = cp.pin['a0'].xya()
                ar = cp.pin['bp'].xya()

            elif (coupler_type=='bend' or coupler_type=='b' or coupler_type=='BDC'):
                cp = circle(xs=xs,radius=R_cp, width = w_cp, theta_start = 270-dAc/2, theta_stop=270+dAc/2, n_points = 1024).cell.put(0,R_cp,0)
                al = cp.pin['a1'].xya()
                ar = cp.pin['b1'].xya()
            else :
                al = nd.Pin().put(0,0,0)
                ar = nd.Pin().put(0,0,0)

            if (attach_type=='circular'):
                attach = circle(xs=xs,radius=R_att,width=w_cp,theta_start=90,theta_stop=90+dAc/2).cell
                bl = attach.put('a1',al)
                br = attach.put('a1',ar,flip=1)
                dW = w_wg-w_cp
                tr = nd.taper(width1=br.pin['b1'].width,width2=w_wg,xs=xs,length=np.abs(dW/np.tan(2/180*np.pi))).put(br.pin['b1'])
                tl = nd.taper(width1=br.pin['b1'].width,width2=w_wg,xs=xs,length=np.abs(dW/np.tan(2/180*np.pi))).put(bl.pin['b1'])

                nd.Pin(name='a1',pin=tl.pin['b0']).put()
                nd.Pin(name='b1',pin=tr.pin['b0']).put()
                
            else:
                attach = spiral_single(R0=R_cp,R1=R_att,w0=w_cp,w1=w_wg,xs=xs,angle=theta_attach-dAc/2).cell
                el = attach.put(al,flip=1)
                er = attach.put(ar)

                bend = spiral_dual(R0=R_att,R1=5,R2=R_wg,w0=w_wg,w1=w_wg,angle1=theta_attach/2,angle2=theta_attach/2).cell
                br = bend.put('a0',el.pin['b0'].xya(),flip=0)
                bl = bend.put('a0',er.pin['b0'].xya(),flip=1)

                nd.Pin(name='a1',pin=bl.pin['b0']).put()
                nd.Pin(name='b1',pin=br.pin['b0']).put()

            ## placing the attching waveguide ##

        #self.sz = [dLx+2*euler_bend.sz[0],dLy+2*euler_bend.sz[1]]
        self.cell = C