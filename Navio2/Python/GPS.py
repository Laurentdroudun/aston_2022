import navio2.util
import navio2.ublox


def init_gps() :
    ubl = navio2.ublox.UBlox("spi:0.0", baudrate=5000000, timeout=2)

    ubl.configure_poll_port()
    ubl.configure_poll(navio2.ublox.CLASS_CFG, navio2.ublox.MSG_CFG_USB)
    #ubl.configure_poll(navio2.ublox.CLASS_MON, navio2.ublox.MSG_MON_HW)

    ubl.configure_port(port=navio2.ublox.PORT_SERIAL1, inMask=1, outMask=0)
    ubl.configure_port(port=navio2.ublox.PORT_USB, inMask=1, outMask=1)
    ubl.configure_port(port=navio2.ublox.PORT_SERIAL2, inMask=1, outMask=0)
    ubl.configure_poll_port()
    ubl.configure_poll_port(navio2.ublox.PORT_SERIAL1)
    ubl.configure_poll_port(navio2.ublox.PORT_SERIAL2)
    ubl.configure_poll_port(navio2.ublox.PORT_USB)
    ubl.configure_solution_rate(rate_ms=1000)

    ubl.set_preferred_dynamic_model(None)
    ubl.set_preferred_usePPP(None)

    ubl.configure_message_rate(navio2.ublox.CLASS_NAV, navio2.ublox.MSG_NAV_POSLLH, 1)
    ubl.configure_message_rate(navio2.ublox.CLASS_NAV, navio2.ublox.MSG_NAV_PVT, 1)
    ubl.configure_message_rate(navio2.ublox.CLASS_NAV, navio2.ublox.MSG_NAV_STATUS, 1)
    ubl.configure_message_rate(navio2.ublox.CLASS_NAV, navio2.ublox.MSG_NAV_SOL, 1)
    ubl.configure_message_rate(navio2.ublox.CLASS_NAV, navio2.ublox.MSG_NAV_VELNED, 1)
    ubl.configure_message_rate(navio2.ublox.CLASS_NAV, navio2.ublox.MSG_NAV_SVINFO, 1)
    ubl.configure_message_rate(navio2.ublox.CLASS_NAV, navio2.ublox.MSG_NAV_VELECEF, 1)
    ubl.configure_message_rate(navio2.ublox.CLASS_NAV, navio2.ublox.MSG_NAV_POSECEF, 1)
    ubl.configure_message_rate(navio2.ublox.CLASS_RXM, navio2.ublox.MSG_RXM_RAW, 1)
    ubl.configure_message_rate(navio2.ublox.CLASS_RXM, navio2.ublox.MSG_RXM_SFRB, 1)
    ubl.configure_message_rate(navio2.ublox.CLASS_RXM, navio2.ublox.MSG_RXM_SVSI, 1)
    ubl.configure_message_rate(navio2.ublox.CLASS_RXM, navio2.ublox.MSG_RXM_ALM, 1)
    ubl.configure_message_rate(navio2.ublox.CLASS_RXM, navio2.ublox.MSG_RXM_EPH, 1)
    ubl.configure_message_rate(navio2.ublox.CLASS_NAV, navio2.ublox.MSG_NAV_TIMEGPS, 5)
    ubl.configure_message_rate(navio2.ublox.CLASS_NAV, navio2.ublox.MSG_NAV_CLOCK, 5)
    #ubl.configure_message_rate(navio2.ublox.CLASS_NAV, navio2.ublox.MSG_NAV_DGPS, 5)
    return ubl

def gps(ubl) :
    msg = ubl.receive_message()
    if msg is None:
        if opts.reopen:
            ubl.close()
            ubl = navio2.ublox.UBlox("spi:0.0", baudrate=5000000, timeout=2)
        # print(empty)
    if msg.name() == "NAV_POSLLH":
        # lon_lat,vit = [int(str(msg).split(",")[1][11:]),int(str(msg).split(",")[2][10:])],str(msg)
        vit=str(msg)
        return vit                       #Returns the longitude and the latitude 
    if msg.name() == "NAV_STATUS":
        outstr = str(msg).split(",")[1:2]
        outstr = "".join(outstr)

if __name__=="__main__" :
    while True :
        ubl=init_gps()
        vit=gps(ubl)
        print("lon_lat : #, vitesse : {}".format(vit))