def testWatchdog():
	#from pylogix import PLC
	import plcComm
	import time
	
	#comm = PLC()
	#comm.IPAddress = '192.168.1.100'
	plcComm.init()	
	readtagstring = 'Program:PC_CommonPart.Exch_PLC_SMB.PLC_WATCHDOG'		
	writetagstring = 'Program:PC_CommonPart.Exch_SMB_PLC.SMB_WATCHDOG'
	while(True):
		
		ret = plcComm.readTag('PLC_WATCHDOG')
		#ret = comm.Read(readtagstring)		
		plcComm.writeTag('SMB_WATCHDOG', ret.Value)
		#comm.Write(writetagstring, ret.Value)		
		time.sleep(1)			
	#comm.Close()
	plcComm.end()
if __name__ == '__main__':
	testWatchdog()

