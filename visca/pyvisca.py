from ctypes import (CDLL, 
                    Structure, 
                    c_char_p, 
                    c_int, 
                    POINTER, 
                    byref)

_lib = CDLL("./libdvisca.so")

class visca_interface(Structure):
    pass
visca_interface_p = POINTER(visca_interface)
visca_interface_pp = POINTER(visca_interface_p)
c_int_p = POINTER(c_int)

_lib.visca_alloc_init_if.argtype = ()
_lib.visca_alloc_init_if.restype = visca_interface_p
_lib.visca_free_if.argtype = (visca_interface_pp)
_lib.visca_free_if.restype = None

_lib.visca_open_if.argtype = (visca_interface_p, c_char_p)
_lib.visca_open_if.restype = c_int
_lib.visca_close_if.argtype = (visca_interface_p)
_lib.visca_close_if.restype = c_int

_arg_types = (
    (visca_interface_p),
    (visca_interface_p, c_int),
    (visca_interface_p, c_int, c_int, c_int),
    (visca_interface_p, c_int, c_int, c_int, c_int),
    (visca_interface_p, c_int_p),
    (visca_interface_p, c_int_p, c_int_p),
)
_method_tab = (
    ("visca_zoom_stop",
     "visca_zoom_tele",
     "visca_zoom_wide",
     "visca_pantilt_home",
     "visca_pantilt_reset",
     "visca_pantilt_stop",
     ),
    ("visca_zoom_tele_speed",
     "visca_zoom_wide_speed",
     "visca_zoom_direct",
     ),
    ("visca_pantilt_dir",
     ),
    ("visca_pantilt_absolute_pos",
     "visca_pantilt_relative_pos",
     ),
    ("visca_inq_zoom_pos",
     ),
    ("visca_inq_pantilt_pos",
     ),
)

for i, v in enumerate(_arg_types):
    for j in _method_tab[i]:
    	getattr(_lib, j).argtype = v
    	getattr(_lib, j).restype = c_int

VISCA_ERR = -1

PT_UP = 0x0301
PT_DOWN = 0x0302
PT_LEFT = 0x0103
PT_RIGHT = 0x0203
PT_UPLEFT = 0x0101
PT_UPRIGHT = 0x0201
PT_DOWNLEFT = 0x0102
PT_DOWNRIGHT = 0x0202

def check(x):
    if x == VISCA_ERR:
        raise "VISCA_ERR"

class ViscaInterface:
    def __init__(self):
        self.pif = _lib.visca_alloc_init_if()
        if not self.pif:
            raise "NULL Pointer"
        self.opened = False

    def open(self, dev_name):
        if self.opened:
            return

        check(_lib.visca_open_if(self.pif, dev_name))
        self.opened = True

    def close(self):
        if not self.opened:
            return

        check(_lib.visca_close_if(self.pif))
        _lib.visca_free_if(byref(self.pif))
        self.opened = False

    def zoom_stop(self):
        check(_lib.visca_zoom_stop(self.pif))
    def zoom_tele(self):
        check(_lib.visca_zoom_tele(self.pif))
    def zoom_wide(self):
        check(_lib.visca_zoom_wide(self.pif))
    def pantilt_home(self):
        check(_lib.visca_pantilt_home(self.pif))
    def pantilt_reset(self):
        check(_lib.visca_pantilt_reset(self.pif))
    def pantilt_stop(self):
        check(_lib.visca_pantilt_stop(self.pif))
    def zoom_tele_speed(self, speed):
        check(_lib.visca_zoom_tele_speed(self.pif, speed))
    def zoom_wide_speed(self, speed):
        check(_lib.visca_zoom_wide_speed(self.pif, speed))
    def zoom_direct(self, pos):
        check(_lib.visca_zoom_direct(self.pif, pos))

    def pantilt_dir(self, pan_speed, tilt_speed, direction):
        check(_lib.visca_pantilt_dir(self.pif, pan_speed, tilt_speed, 
                                     direction))

    def pantilt_absolute_pos(self, pan_speed, tilt_speed, pan_pos, tilt_pos):
        check(_lib.visca_pantilt_absolute_pos(self.pif, pan_speed,
                                              tilt_speed, pan_pos, tilt_pos))

    def pantilt_relative_pos(self, pan_speed, tilt_speed, pan_pos, tilt_pos):
        check(_lib.visca_pantilt_relative_pos(self.pif, pan_speed,
                                              tilt_speed, pan_pos, tilt_pos))
    def inq_zoom_pos(self):
        zoom_pos = c_int()
        check(_lib.visca_inq_zoom_pos(self.pif, byref(zoom_pos)))
        return zoom_pos.value
    
    def inq_pantilt_pos(self):
        pan_pos, tilt_pos = c_int(), c_int()
        check(_lib.visca_inq_pantilt_pos(self.pif, byref(pan_pos), 
                                         byref(tilt_pos)))
        return pan_pos.value, tilt_pos.value
    
if __name__ == "__main__":    
    iface = ViscaInterface()
    iface.open("/dev/ttyS0")
    
    iface.zoom_direct(2000)
    print iface.inq_zoom_pos()

    iface.pantilt_absolute_pos(20,20,100,100)
    print iface.inq_pantilt_pos()

    iface.close()
