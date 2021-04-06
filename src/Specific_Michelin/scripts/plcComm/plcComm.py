from pylogix import PLC

def test():
	from pylogix import PLC
	comm = PLC()
	comm.IPAddress = '192.168.1.100'
	ret = comm.Read('Program:PC_CommonPart.R010_ConsVitesse')	
	return ret.Value

def mult(a,b):
	print('Python function mult() called')
	c = a * b
	return c

def testReturn():
	return 42

def testTextArgument(textArg):
	print(textArg)
	return

def testAppend(textArg):
	enTete = "Program:PC_CommonPart."
	result = enTete + textArg
	print(result)
	return

def init():
	global comm
	global testInt
	comm = PLC()
	comm.IPAddress = '192.168.1.100'
	print('init done')

def readTag(tagName):	
	enTete = "Program:PC_CommonPart.Exch_PLC_SMB."
	tagstring = enTete + tagName
	#print('trying to read tag ' + tagstring)
	ret = comm.Read(tagstring)
	return ret

def writeTag(tagName, value):
	enTete = "Program:PC_CommonPart.Exch_SMB_PLC."
	tagstring = enTete + tagName		
	#print('trying to write ' + str(value) + ' into ' + tagstring) 	
	comm.Write(tagstring, value)
	
def end():
	comm.Close()

#python pour gerer moi meme le PLC
#	from pylogix import PLC
#	comm = PLC()
#	comm.IPAddress = '192.168.1.100'
#	ret = comm.Write('Program:PC_CommonPart.R077_SMB.PLC_SMB_request', 1) #init
#
#   comm.Write('Program:PC_CommonPart.Exch_SMB_PLC.GRIPPER_D_1_in.b_MoveToWork', 1)
#   comm.Read('Program:PC_CommonPart.Exch_PLC_SMB.GRIPPER_D_1_out.b_BasePosition')
