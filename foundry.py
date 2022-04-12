import nazca as nd
import numpy as np

class AMF_tapout :
    LAYER_WG_RIB = 10
    LAYER_WG_GRATING_COUPLER = 11
    LAYER_WG_SLAB = 12
    LAYER_METAL = 105
    LAYER_HEATER = 115
    LAYER_VIA2META1 = 120
    LAYER_METAL_2 = 125
    LAYER_METAL_PAD = 13
    LAYER_WG_SI3N4 = 54
    STD_SMWG_WIDTH = 0.45

    lib_path = 'mx_lib\\GDS\\'

    def __init__ (self):
        nd.add_layer(name='Rib', layer=self.LAYER_WG_RIB)
        nd.add_layer2xsection(xsection='Rib', layer='Rib')
        self.XS_WG_RIB = 'Rib'
        nd.add_layer(name='Slab', layer=self.LAYER_WG_SLAB)
        nd.add_layer2xsection(xsection='Slab', layer='Rib')
        nd.add_layer2xsection(xsection='Slab', layer='Slab',leftedge=(0.5, 2), rightedge=(-0.5, -2))
        self.XS_WG_SLAB = 'Slab'
        nd.add_layer(name='Heater', layer=self.LAYER_HEATER)
        nd.add_layer2xsection(xsection='Heater', layer='Heater')
        self.XS_HEATER = 'Heater'

        nd.add_layer(name='METAL', layer=self.LAYER_METAL)
        nd.add_layer2xsection(xsection='METAL', layer='METAL')
        self.XS_METAL = 'METAL'

    W_METAL_MIN = 5
    SPACING_METAL_MIN = 8
    W_HEATER_MIN = 3

class ANT_MPW :
    STD_SMWG_WIDTH = 0.45
    SLAB_GROWTH = 3
    W_METAL_MIN = 5
    SPACING_HEATER_MIN = 5
    SPACING_METAL_MIN = 8
    W_HEATER_MIN = 3

    lib_path = 'GDS_lib\\'

    def __init__ (self):
        nd.add_xsection(name='strip')
        nd.add_layer(name='STRIP_COR', layer=(1,0)) ## defining the strip waveguide core area
        nd.add_layer2xsection(xsection='strip', layer='STRIP_COR')
        self.LAYER_STRIP_COR = 'STRIP_COR' # 31,1
        self.XS_WG_STRIP =  'strip' # 31,1

        nd.add_layer(name='HEATER', layer=(11,0))
        nd.add_layer2xsection(xsection='heater', layer='HEATER')
        self.XS_HEATER = 'heater'
        self.LAYER_HEATER = 'HEATER' # 31,1

        nd.add_layer(name='METAL', layer=(12,1))
        nd.add_layer2xsection(xsection='metal', layer='METAL')
        self.XS_METAL = 'metal'
        self.LAYER_METAL = 'METAL'

        nd.add_layer(name='PAD_OPEN', layer=(13,1))
        nd.add_layer2xsection(xsection='pad_open', layer='PAD_OPEN')
        self.XS_PAD_OPEN = 'pad_open'
        self.LAYER_PAD_OPEN = 'PAD_OPEN'




class CUMEC_ACTIVE :

    STD_SMWG_WIDTH = 0.45
    SLAB_GROWTH = 2
    W_METAL_MIN = 1
    SPACING_HEATER_MIN = 1
    SPACING_METAL_MIN = 1.5
    W_HEATER_MIN = 1

    lib_path = 'GDS_lib\\'

    def __init__ (self):
        nd.add_xsection(name='strip')
        nd.add_layer(name='STRIP_COR', layer=(31,1)) ## defining the strip waveguide core area
        nd.add_layer(name='STRIP_ClD', layer=(31,2)) ## defining the etched side for strip waveguide core area
        nd.add_layer(name='STRIP_TRE', layer=(31,3)) ## defining the dense trench of the full etched area
        nd.add_layer(name='STRIP_HOL', layer=(31,4)) ## defining the dense holes of the full etched area
        nd.add_layer2xsection(xsection='strip', layer='STRIP_COR')
        nd.add_layer2xsection(xsection='strip', layer='STRIP_ClD',leftedge=(0.5, self.SLAB_GROWTH), rightedge=(-0.5, -self.SLAB_GROWTH))
        self.LAYER_STRIP_COR = 'STRIP_COR' # 31,1
        self.LAYER_STRIP_CLD = 'STRIP_CLD' # 31,2
        self.LAYER_STRIP_TRE = 'STRIP_TRE' # 31,3
        self.LAYER_STRIP_HOL = 'STRIP_HOL' # 31,4
        self.XS_WG_STRIP =  'strip' # 31,1



        nd.add_xsection(name='rib_s')
        nd.add_layer(name='SRIB_COR', layer=(32,1)) ## defining the Rib area with 70nm etched waveguide core area
        nd.add_layer(name='SRIB_CLD', layer=(32,2)) ## defining the etched side for strip waveguide core area
        nd.add_layer(name='SRIB_TRE', layer=(32,3)) ## defining the etched side for strip waveguide core area
        nd.add_layer(name='SRIB_HOL', layer=(33,4)) ## defining the etched side for strip waveguide core area
        nd.add_layer2xsection(xsection='rib_s', layer='SRIB_COR')
        nd.add_layer2xsection(xsection='rib_s', layer='SRIB_CLD',leftedge=(0.5, self.SLAB_GROWTH), rightedge=(-0.5, -self.SLAB_GROWTH))
        self.XS_WG_SRIB = 'rib_s'
        self.LAYER_SRIB_COR = 'SRIB_COR ' # 31,1
        self.LAYER_SRIB_CLD = 'SRIB_CLD ' # 31,1
        self.LAYER_SRIB_TRE = 'SRIB_TRE ' # 31,1
        self.LAYER_SRIB_HOL = 'SRIB_HOL ' # 31,1

        ''' Default Rib waveguide is the deep etched waveguide '''
        nd.add_xsection(name='rib')
        nd.add_layer(name='DRIB_COR', layer=(33,1)) ## defining the Rib area with 150nm etched waveguide core area
        nd.add_layer(name='DRIB_CLD', layer=(33,2)) ## defining the Rib area with 150nm etched waveguide core area
        nd.add_layer(name='DRIB_TRE', layer=(33,3)) ## defining the Rib area with 150nm etched waveguide core area
        nd.add_layer(name='DRIB_HOL', layer=(33,4)) ## defining the Rib area with 150nm etched waveguide core area
        nd.add_layer2xsection(xsection='rib', layer='DRIB_COR')
        nd.add_layer2xsection(xsection='rib', layer='DRIB_CLD',leftedge=(0.5, self.SLAB_GROWTH), rightedge=(-0.5, -self.SLAB_GROWTH))
        self.XS_WG_RIB = 'rib'
        self.LAYER_DRIB_COR = 'DRIB_COR' # 31,1
        self.LAYER_DRIB_CLD = 'DRIB_CLD' # 31,1
        self.LAYER_DRIB_TRE = 'DRIB_TRE' # 31,1
        self.LAYER_DRIB_HOL = 'DRIB_HOL' # 31,1

        nd.add_xsection(name='rib_d')
        nd.add_layer(name='DRIB_COR', layer=(33,1)) ## defining the Rib area with 150nm etched waveguide core area
        nd.add_layer(name='DRIB_CLD', layer=(33,2)) ## defining the Rib area with 150nm etched waveguide core area
        nd.add_layer(name='DRIB_TRE', layer=(33,3)) ## defining the Rib area with 150nm etched waveguide core area
        nd.add_layer(name='DRIB_HOL', layer=(33,4)) ## defining the Rib area with 150nm etched waveguide core area
        nd.add_layer2xsection(xsection='rib_d', layer='DRIB_COR')
        nd.add_layer2xsection(xsection='rib_d', layer='DRIB_CLD',leftedge=(0.5, self.SLAB_GROWTH), rightedge=(-0.5, -self.SLAB_GROWTH))
        self.XS_WG_RIB_2 = 'rib_d'
        self.LAYER_DRIB_COR = 'DRIB_COR' # 31,1
        self.LAYER_DRIB_CLD = 'DRIB_CLD' # 31,1
        self.LAYER_DRIB_TRE = 'DRIB_TRE' # 31,1
        self.LAYER_DRIB_HOL = 'DRIB_HOL' # 31,1

        nd.add_layer(name='HEATER', layer=(19,0))
        nd.add_layer2xsection(xsection='heater', layer='HEATER')
        self.XS_HEATER = 'heater'
        self.LAYER_HEATER = 'HEATER' # 31,1

        nd.add_layer(name='VIA_H2M', layer=(50,0))
        nd.add_layer2xsection(xsection='via_h2m', layer='VIA_H2M')
        self.XS_VIA_H2M = 'via_h2m'
        self.LAYER_VIA_H2M = 'VIA_H2M' # the contact layer in CUMEC definition, connecting the metal to heater
        self.W_VIA_H2M = 0.25
        self.SPACING_VIA_H2M = 0.4

        nd.add_layer(name='METAL', layer=(11,1))
        nd.add_layer2xsection(xsection='metal', layer='METAL')
        self.XS_METAL = 'metal'
        self.LAYER_METAL = 'METAL'

        nd.add_layer(name='METAL_2', layer=(12,1))
        nd.add_layer2xsection(xsection='metal_2', layer='METAL_2')
        self.XS_METAL_2 = 'metal_2'
        self.LAYER_METAL_2 = 'METAL_2'

        nd.add_layer(name='VIA_M2M', layer=(51,0))
        nd.add_layer2xsection(xsection='via_m2m', layer='VIA_M2M')
        self.XS_METAL_2 = 'via_m2m'
        self.LAYER_METAL_2 = 'VIA_M2M'

        nd.add_xsection(name='thermal_isl')
        nd.add_layer(name='THERMAL_ISL', layer=(67,0))
        nd.add_layer2xsection(xsection='thermal_isl', layer='THERMAL_ISL')
        nd.add_layer2xsection(xsection='thermal_isl', layer='STRIP_ClD',leftedge=(0.5, 1), rightedge=(-0.5, -1)) ## Thermal trench must be put inside the Cladding
        self.XS_THERMAL_ISL = 'thermal_isl'
        self.LAYER_THERMAL_ISL = 'THERMAL_ISL'

        nd.add_xsection(name='bond_pad')
        nd.add_layer(name='BOND_PAD', layer=(20,0))
        nd.add_layer2xsection(xsection='bond_pad', layer='BOND_PAD')
        self.XS_BOND_PAD = 'bond_pad'
        self.LAYER_BOND_PAD = 'BOND_PAD'

        nd.add_xsection(name='pad_open')
        nd.add_layer(name='PAD_OPEN', layer=(63,0))
        nd.add_layer2xsection(xsection='pad_open', layer='PAD_OPEN')
        self.XS_PAD_OPEN = 'pad_open'
        self.LAYER_PAD_OPEN = 'PAD_OPEN'