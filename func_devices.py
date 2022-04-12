import nazca as nd
import numpy as np

from .structures import *
from .unit_devices import *



'''
# =================================================================
# @ File : <mx_devices_lib.py>
# @ Device: Standard ring resonator
# @           
# =================================================================
'''

class STD_Ring_resonator :
    def __init__ (self,tapout,radius=10,width=0.45,coupler1_angle=0,xs_wg='strip',coupler1_width=0.45,gap1=0.22,w_heater=0,w_metal=0,coupler2_angle=0,coupler2_width=0,gap2=0.45,gds_lib_name='',gds_lib_path='GDS_lib\\'): 

        '''
        Initialization of a Standard circular ring resonator with/ without cricular heater
        Support two types of resonator, single coupler and dual coupler


        Paras : 
        0. gds_lib_name             
            - gds_lib_name          ['str']     (Default: null)
                If this is not null, then the function will load the gds file as a cell
        1. ring part
            - radius                [um]        (Default: 10um)
                Radius of the resonator, ring center to waveguide center
            - width                 [um]        (Default: 0.45um)
                Width of the resonator
            - wg_type               ['strip','rib','mod'] (Default : 'strip')
                waveguide type, strip waveguide, or shallow etched waveguide

        2. heater part
            - w_heater              [um]        (Default: 3.5um)
                heater width
            - w_metal               [um]        (Default: 5um)
                metal width attached to the heater

        3. coupler 1 part, the lower coupler
            - coupler1_angle        [arc]        (Default: 0)
                the angle to ring center of the bent directional coupler used for coupling
            - coupler1_width        [um]        (Default: 0.45um)
                the width of the bent directional coupler
            - gap1                  [um]        (Default: 0.22um)
                the gap of the bent directional coupler

        4. coupler 2 part, the upper coupler  (Default disabled)
            - coupler2_angle        [arc]        (Default: 0)
                the angle to ring center of the bent directional coupler used for coupling
            - coupler2_width        [um]        (Default: 0.45um)
                the width of the bent directional coupler
            - gap2                  [um]        (Default: 0.22um)
                the gap of the bent directional coupler

        '''
        if (gds_lib_name==''):
            with nd.Cell(instantiate=False) as C:
                
                ## adding the major ring ##
                nd.bend(xs=xs_wg,radius=radius,width=width,angle=360).put(0,-radius,0)

                ## The lower coupling bus ##
                _radius_coupler_ = radius+width/2+gap1+coupler1_width/2
                nd.bend(xs=xs_wg,radius=_radius_coupler_,width=coupler1_width,angle=coupler1_angle/2).put(0,-_radius_coupler_,0)
                nd.bend(xs=xs_wg,radius=_radius_coupler_,width=coupler1_width,angle=coupler1_angle/2).put(flip=1)
                temp = nd.strt(xs=xs_wg,length=radius-radius*np.sin(coupler1_angle/2),width=coupler1_width).put(flip=0)
                nd.Pin(name='b1',pin=temp.pin['b0']).put()

                nd.bend(xs=xs_wg,radius=_radius_coupler_,width=coupler1_width,angle=coupler1_angle/2).put(0,-_radius_coupler_,180,flip=1)
                nd.bend(xs=xs_wg,radius=_radius_coupler_,width=coupler1_width,angle=coupler1_angle/2).put(flip=0)
                temp = nd.strt(xs=xs_wg,length=radius-radius*np.sin(coupler1_angle/2),width=coupler1_width).put(flip=0)
                nd.Pin(name='a1',pin=temp.pin['b0']).put()

                _heater_upper_ = 90
                w_pad = np.max([tapout.W_METAL_MIN,w_metal])
                _theta_end_ = 45 + w_pad*np.sin(np.pi/4)/radius*180/np.pi
                ## The upper coupling bus ##
                if coupler2_width >0:
                    nd.bend(xs=xs_wg,radius=radius,width=width,angle=360).put(0,-radius,0)
                    _radius_coupler_ = radius+width/2+gap2+coupler2_width/2
                    nd.bend(xs=xs_wg,radius=_radius_coupler_,width=coupler2_width,angle=coupler2_angle/2).put(0, _radius_coupler_,0,flip=1)
                    nd.bend(xs=xs_wg,radius=_radius_coupler_,width=coupler2_width,angle=coupler2_angle/2).put(flip=0)
                    temp = nd.strt(xs=xs_wg,length=radius-radius*np.sin(coupler2_angle/2),width=coupler1_width).put(flip=0)
                    nd.Pin(name='b2',pin=temp.pin['b0']).put()

                    nd.bend(xs=xs_wg,radius=_radius_coupler_,width=coupler2_width,angle=coupler2_angle/2).put(0, _radius_coupler_,180,flip=0)
                    nd.bend(xs=xs_wg,radius=_radius_coupler_,width=coupler2_width,angle=coupler2_angle/2).put(flip=1)
                    temp = nd.strt(xs=xs_wg,length=radius-radius*np.sin(coupler2_angle/2),width=coupler1_width).put(flip=0)
                    nd.Pin(name='a2',pin=temp.pin['b0']).put()
                    if (w_heater>0):
                        _heater_upper_ = 180 - _theta_end_

                ## == adding heaters == ##
                if (w_heater>0):
                    w_heater_cur = np.max([w_heater,tapout.W_HEATER_MIN])

                    circle(xs=tapout.XS_HEATER,radius=radius,width=w_heater_cur,theta_start=_heater_upper_,theta_stop=180+_theta_end_).cell.put(0,0,0)
                    circle(xs=tapout.XS_HEATER,radius=radius,width=w_heater_cur,theta_start=-_theta_end_,theta_stop=180-_heater_upper_).cell.put(0,0,0)

                    x_pad = w_pad/2-radius*np.cos(45/180*np.pi)
                    y_pad = -radius*np.sin(45/180*np.pi)-w_pad/2

                    if coupler2_width >0:
                        nd.strt(length=w_pad,width=w_pad,xs=tapout.XS_HEATER).put(x_pad,-y_pad+w_pad/2,-90)
                        nd.strt(length=w_pad,width=w_pad,xs=tapout.XS_HEATER).put(-x_pad,-y_pad+w_pad/2,-90)

                        nd.strt(length=w_pad,width=w_pad,xs=tapout.XS_METAL).put(x_pad,-y_pad+w_pad/2,-90)
                        nd.strt(length=w_pad,width=w_pad,xs=tapout.XS_METAL).put(-x_pad,-y_pad+w_pad/2,-90)
                        if (hasattr(tapout,'XS_VIA_H2M')): ## placing vias
                            vias = Vias(xs_via=tapout.XS_VIA_H2M,area=[w_pad,w_pad],via_sz=[tapout.W_VIA_H2M,tapout.W_VIA_H2M],via_spacing=[tapout.SPACING_VIA_H2M,tapout.SPACING_VIA_H2M]).cell
                            vias.put(x_pad,-y_pad)
                            vias.put(-x_pad,-y_pad)
                        nd.strt(length=radius*2*np.cos(45/180*np.pi),width=w_pad,xs=tapout.XS_METAL).put(-radius*np.cos(45/180*np.pi),w_pad/2+radius*np.sin(45/180*np.pi),0)

                    if (hasattr(tapout,'XS_VIA_H2M')):## placing vias
                        vias = Vias(xs_via=tapout.XS_VIA_H2M,area=[w_pad,w_pad],via_sz=[tapout.W_VIA_H2M,tapout.W_VIA_H2M],via_spacing=[tapout.SPACING_VIA_H2M,tapout.SPACING_VIA_H2M]).cell
                        vias.put(x_pad,y_pad)
                        vias.put(-x_pad,y_pad)

                    pad_l = nd.strt(length=w_pad,width=w_pad,xs=tapout.XS_HEATER).put(x_pad,y_pad+w_pad/2,-90)
                    pad_r = nd.strt(length=w_pad,width=w_pad,xs=tapout.XS_HEATER).put(-x_pad,y_pad+w_pad/2,-90)

                    pad_l = nd.strt(length=w_pad,width=w_pad,xs=tapout.XS_METAL).put(x_pad,y_pad+w_pad/2,-90)
                    pad_r = nd.strt(length=w_pad,width=w_pad,xs=tapout.XS_METAL).put(-x_pad,y_pad+w_pad/2,-90)
                    nd.Pin(name='e1',pin=pad_l.pin['b0']).put()
                    nd.Pin(name='e2',pin=pad_r.pin['b0']).put()

            self.cell = C

        # else:
        #     self.cell = mgl.mx_lib_load(gds_lib_path,gds_lib_name)


class Euler_racetrack_CROWs :
    def __init__ (self,tapout,gaps=[0.22,0.22,0.22],w1=[0.45],w0=[0.45],R1=[10],R0=[10],R2=[10],dLx=0,dLy=0,w_cp=0.35,R_cp_max=15,theta_cp_attach=30,w_wg=0.45,dC=0,cp_type='BDC',xs_wg='strip',bend_type='dual',heater_type='connected',w_heater=3.5,w_metal=5,gds_lib_name='',gds_lib_path='GDS_lib\\'):
        
        '''
        Initialization of a Standard circular ring resonator with/ without cricular heater
        Support two types of resonator, single coupler and dual coupler


        Paras : 
        0. gds_lib_name             
            - gds_lib_name          ['str']     (Default: null)
                If this is not null, then the function will load the gds file as a cell
        1. ring part
            - radius                [um]        (Default: 10um)
                Radius of the resonator, ring center to waveguide center
            - width                 [um]        (Default: 0.45um)
                Width of the resonator
            - wg_type               ['strip','rib','mod'] (Default : 'strip')
                waveguide type, strip waveguide, or shallow etched waveguide

        2. heater part
            - w_heater              [um]        (Default: 3.5um)
                heater width
            - w_metal               [um]        (Default: 5um)
                metal width attached to the heater

        3. coupler 1 part, the lower coupler
            - coupler1_angle        [arc]        (Default: 0)
                the angle to ring center of the bent directional coupler used for coupling
            - coupler1_width        [um]        (Default: 0.45um)
                the width of the bent directional coupler
            - gap1                  [um]        (Default: 0.22um)
                the gap of the bent directional coupler

        4. coupler 2 part, the upper coupler  (Default disabled)
            - coupler2_angle        [arc]        (Default: 0)
                the angle to ring center of the bent directional coupler used for coupling
            - coupler2_width        [um]        (Default: 0.45um)
                the width of the bent directional coupler
            - gap2                  [um]        (Default: 0.22um)
                the gap of the bent directional coupler

        '''

        self.R0 = R0
        self.R1 = R1
        self.R2 = R2
        self.w0 = w0
        self.w1 = w1
        self.w_cp = w_cp
        self.w_wg = w_wg
        self.dC = dC
        if (gds_lib_name==''):
            with nd.Cell(instantiate=False) as C:

                order = len(gaps) - 1 # the order of the crow

                w_pad = np.max([tapout.W_METAL_MIN,w_metal])
                w_ht = np.max([tapout.W_HEATER_MIN,w_heater])
                if (len(w1)==1):
                    w1 = w1*np.ones((len(gaps)))
                    w0 = w0*np.ones((len(gaps)))
                    R1 = R1*np.ones((len(gaps)))
                    R0 = R0*np.ones((len(gaps)))
                    R2 = R2*np.ones((len(gaps)))
                    dLx = dLx*np.ones((len(gaps)))
                    dLy = dLy*np.ones((len(gaps)))

                idx = np.linspace(0,order-1,order)
                _y_init_ = gaps[0]/2+w_cp/2

                # ''' Placing the inner crows by order ''' # 
                for _idx_ in idx:
                    _idx_ = int(_idx_)
                    if (bend_type=='dual'):
                        rck_euler = spiral_dual(xs=xs_wg,w1=w1[_idx_],w0=w0[_idx_],R1=R1[_idx_],R0=R0[_idx_],R2=R2[_idx_])
                    else :
                        rck_euler = spiral_single(xs=xs_wg,w1=w1[_idx_],w0=w0[_idx_],R1=R1[_idx_],R0=R0[_idx_])
                        
                    rack_cur = rck_euler.racetrack(dLx=dLx[_idx_],dLy=dLy[_idx_])
                    _y_rack_ = _y_init_ + rck_euler.rck_sz[1]/2 +w0[_idx_]/2+gaps[_idx_]/2
                    rack_cur.put(0,_y_rack_ ,0)
                    _y_init_ = _y_init_ + rck_euler.rck_sz[1] + w0[_idx_] + gaps[_idx_+1]/2 + gaps[_idx_]/2
                    if (w_heater!=0):

                        _r_heater_ = rck_euler.rck_sz[0]/2
                        circle(xs=tapout.XS_HEATER,radius=_r_heater_,width=w_ht,theta_start=135, theta_stop=180).cell.put(-dLx[_idx_]/2,_y_rack_+dLy[_idx_]/2,0)
                        circle(xs=tapout.XS_HEATER,radius=_r_heater_,width=w_ht,theta_start=180, theta_stop=225).cell.put(-dLx[_idx_]/2,_y_rack_-dLy[_idx_]/2,0)
                        circle(xs=tapout.XS_HEATER,radius=_r_heater_,width=w_ht,theta_start=0, theta_stop=45).cell.put(dLx[_idx_]/2,_y_rack_+dLy[_idx_]/2,0)
                        circle(xs=tapout.XS_HEATER,radius=_r_heater_,width=w_ht,theta_start=-45, theta_stop=0).cell.put(dLx[_idx_]/2,_y_rack_-dLy[_idx_]/2,0)
                        nd.strt(length=dLy[_idx_],width=w_ht,xs=tapout.XS_HEATER).put(-dLx[_idx_]/2-_r_heater_,_y_rack_-dLy[_idx_]/2,90)
                        nd.strt(length=dLy[_idx_],width=w_ht,xs=tapout.XS_HEATER).put( dLx[_idx_]/2+_r_heater_,_y_rack_-dLy[_idx_]/2,90)
                        if (heater_type=='connected'):
                            if (_idx_==0): # starting ring
                                _L_attach_heater_ = 3*w_ht
                                _x_attach_ = -dLx[_idx_]/2-_r_heater_*np.cos(np.pi/4)-(1-np.cos(np.pi/4))*w_ht/2
                                _y_attach_ = _y_rack_-dLy[_idx_]/2-_r_heater_*np.sin(np.pi/4)+w_ht*np.sin(np.pi/4)/2
                                h_dl = nd.strt(length=_L_attach_heater_,width=w_ht,xs=tapout.XS_HEATER).put(_x_attach_,_y_attach_,-90)
                                h_dr = nd.strt(length=_L_attach_heater_,width=w_ht,xs=tapout.XS_HEATER).put(-_x_attach_,_y_attach_,-90)
                            else:
                                _L_attach_heater_ = rck_euler.rck_sz[1]/2-dLy[_idx_]/2- _r_heater_*np.cos(np.pi/4) + w0[_idx_]/2+gaps[_idx_]/2 +w_ht/2
                                _x_attach_ = -dLx[_idx_]/2-_r_heater_*np.cos(np.pi/4)-(1-np.cos(np.pi/4))*w_ht/2
                                _y_attach_ = _y_rack_-dLy[_idx_]/2-_r_heater_*np.sin(np.pi/4)+w_ht*np.sin(np.pi/4)/2
                                nd.strt(length=_L_attach_heater_,width=w_ht,xs=tapout.XS_HEATER).put(_x_attach_,_y_attach_,-90)
                                nd.strt(length=_L_attach_heater_,width=w_ht,xs=tapout.XS_HEATER).put(-_x_attach_,_y_attach_,-90)

                            if (_idx_==order-1): # starting ring
                                _L_attach_heater_ = 3*w_ht
                            else:
                                _L_attach_heater_ = rck_euler.rck_sz[1]/2-dLy[_idx_]/2- _r_heater_*np.cos(np.pi/4) + w0[_idx_]/2+gaps[_idx_]/2 +w_ht/2
                            _x_attach_ = -dLx[_idx_]/2-_r_heater_*np.cos(np.pi/4)-(1-np.cos(np.pi/4))*w_ht/2
                            _y_attach_ = _y_rack_+dLy[_idx_]/2+_r_heater_*np.sin(np.pi/4)-w_ht*np.sin(np.pi/4)/2
                            h_ul = nd.strt(length=_L_attach_heater_,width=w_ht,xs=tapout.XS_HEATER).put(_x_attach_,_y_attach_,90)
                            h_ur = nd.strt(length=_L_attach_heater_,width=w_ht,xs=tapout.XS_HEATER).put(-_x_attach_,_y_attach_,90)
                


                # ''' Placing the electro interface''' # 
                if (w_heater!=0):

                    ## == Placing the bridge in the above == ## 
                    mur = nd.strt(width=w_pad,xs=tapout.XS_HEATER,length=w_pad).put(h_ur.pin['b0'].x-w_ht/2,h_ur.pin['b0'].y+w_pad/2,0)    
                    mul = nd.strt(width=w_pad,xs=tapout.XS_HEATER,length=w_pad).put(h_ul.pin['b0'].x+w_ht/2,h_ul.pin['b0'].y+w_pad/2,180)    
                    nd.strt(width=w_pad,xs=tapout.XS_METAL,length=np.abs(mul.pin['b0'].x-mur.pin['b0'].x)).put(mul.pin['b0'].x,mur.pin['b0'].y,0)    
                                    
                    ## == Placing the output port in the below == ## 
                    mtl = nd.strt(width=w_pad,xs=tapout.XS_HEATER,length=tapout.SPACING_METAL_MIN/2+w_pad).put(h_dl.pin['b0'].x+w_ht/2,h_dl.pin['b0'].y-w_pad/2,180)    
                    mtr = nd.strt(width=w_pad,xs=tapout.XS_HEATER,length=tapout.SPACING_METAL_MIN/2+w_pad).put(h_dr.pin['b0'].x-w_ht/2,h_dr.pin['b0'].y-w_pad/2,0)


                    x_pad = mtl.pin['b0'].x+w_pad/2
                    y_pad = mtl.pin['b0'].y
                    y_pad_up = mul.pin['b0'].y
                    x_pad_up = mul.pin['b0'].x+w_pad/2

                    if (hasattr(tapout,'XS_VIA_H2M')):## placing vias
                        vias = Vias(xs_via=tapout.XS_VIA_H2M,area=[w_pad,w_pad],via_sz=[tapout.W_VIA_H2M,tapout.W_VIA_H2M],via_spacing=[tapout.SPACING_VIA_H2M,tapout.SPACING_VIA_H2M]).cell
                        vias.put(x_pad,y_pad)
                        vias.put(-x_pad,y_pad)
                        vias.put(x_pad_up,y_pad_up)
                        vias.put(-x_pad_up,y_pad_up)

                    mtl = nd.strt(width=w_pad,xs=tapout.XS_METAL,length=w_pad).put(x_pad,y_pad+w_pad/2,-90)    
                    mtr = nd.strt(width=w_pad,xs=tapout.XS_METAL,length=w_pad).put(-x_pad,y_pad+w_pad/2,-90)    
                    nd.Pin(name='e1',pin=mtl.pin['b0']).put()
                    nd.Pin(name='e2',pin=mtr.pin['b0']).put()
                if cp_type=='BDC' or cp_type=='bend' or cp_type=='b':
                    R_cp = R0[0]+w0[0]/2+gaps[0]+w_cp/2
                    cp_d = coupler_input(xs=xs_wg,w_wg=w_wg,w_cp=w_cp,R_att=R_cp_max,R_cp=R_cp,dLc=0,dAc=dC,theta_attach=theta_cp_attach,attach_type='euler',coupler_type = 'BDC').cell.put(0,0,0)
                    cp_u = coupler_input(xs=xs_wg,w_wg=w_wg,w_cp=w_cp,R_att=R_cp_max,R_cp=R_cp,dLc=0,dAc=dC,theta_attach=theta_cp_attach,attach_type='euler',coupler_type = 'BDC').cell.put(0,_y_init_+ w_cp/2+gaps[-1]/2,0,flip=1)

                nd.Pin(name='a1',pin=cp_u.pin['a1']).put()
                nd.Pin(name='a2',pin=cp_d.pin['a1']).put()
                nd.Pin(name='b1',pin=cp_u.pin['b1']).put()
                nd.Pin(name='b2',pin=cp_d.pin['b1']).put()
                ## generating the coupler ##
                

            self.cell = C

        # else:
        #     self.cell = mgl.mx_lib_load(gds_lib_path,gds_lib_name)
 



