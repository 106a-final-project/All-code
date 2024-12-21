import time
import psocScanner as psoc
import struct
import numpy as np
import os
import math
import scipy.fftpack
# import robotiq as r
import winsound

runTime = 20 # 20 for experiment, 4 for testing

frequency = 2500  # Set Frequency To 2500 Hertz
duration = 100  # Set duration in ms, 1000 ms == 1 second

SensorExist = 1
plotShow = 0

# ResultSavingDirectory = '/home/bdml/Tae_Data'
SavingFileName = 'debug'

##################################################
# r.init_Port('COM3')
# ClosingSpeed = 255
# ClosingForce = 150
# Threshold = -100 #15
# time.sleep(2)
# r.activate()
# r.setForce(ClosingForce)
# time.sleep(0.1)
# r.setSpeed(ClosingSpeed)
# time.sleep(0.1)
# CurrentPosition = 100
# r.setPosition(CurrentPosition)
# time.sleep(2)
# print('set position 100')
# CurrentPosition = 150
# r.setPosition(CurrentPosition)
# time.sleep(2)
# print('set position 150')

##################################################
#%%
SensorNum = 1 # This Code can take upto two sensor input. 
SensorAddress = np.array([8, 9]) # if SensorNum == 1, second address will be ignored.
slopeCompensateOn = True

MODE_ONE_PAD = 0    # measures pressure, "super fast" (~kHz), 1 channel
MODE_FOUR_PAD = 1   # measures linear in 4 directions
MODE_NINE_PAD = 2   # measures pressure across "nibs" (each is a cluster of 4 pips)
MODE_INDIVIDUAL = 3 # measures each electrode (hardwired across 4 pips)
MODE_LINEAR_ROTATIONAL = 4  # measures linear (up/down) and rotational (cw/ccw)
MODE_CAPSENSE = 17
MODE_ONE_AND_FOUR =32   
MODE_NINE_AND_INDIV= 33 
MODE_ONE_AND_NINE = 34  
MODE_FOUR_AND_INDIV = 35 
MODE_TOR_NS_AND_INDIV = 36

# individual (coupled rotation in software), individual (coupled rotation+linear in software), rotation-only, rotation + linear

#walnut (20g) = [214,217]
#1/4" acrylic (20g) = [212,213]
#1/8" acrylic with masking tape () = [125]

#1/4" acrylic with nothing on, barely holds at 217 --> [min,max] = [] on software...[217,224] here ... [213] is now better...
#1/4" draftboard with nothing on, barely holds at 216 --> [min,max] = [] on software...[217,224] here
#1/8" walnut with nothing on, barely holds at 218 --> [min,max] = [213,218] on software...[217,224] here

#acrylic: robotiq213
#draftboard: robotiq214
#walnut: robotiq218
sensingMode = MODE_INDIVIDUAL # MODE_INDIVIDUAL vs. MODE_LINEAR_ROTATIONAL
material = 'draftboard' # acrylic draftboard walnut
print("Mode: "+str(sensingMode)+', '+material)

if material == 'acrylic': #50g
    close_min = 213
    close_max = 213
    addClose = 2 # how much to increase the close
    slip_threshold = 100
elif material == 'walnut': #20g
    close_min = 217
    close_max = 217
    addClose = 3 # how much to increase the close
    slip_threshold = 100
elif material == 'draftboard': #50g
    close_min = 214
    close_max = 214
    addClose = 0 # how much to increase the close
    slip_threshold = 100
if sensingMode == MODE_LINEAR_ROTATIONAL:
    slip_threshold = slip_threshold * 10

CurrentPosition = close_min
# r.setPosition(close_min)
print('set position at MIN: '+str(close_min))





IsTwoModeMerged = False
if sensingMode == MODE_ONE_PAD:
    SamplingFreq = 2e3
elif sensingMode == MODE_FOUR_PAD:
    SamplingFreq = 600; 
elif sensingMode == MODE_NINE_PAD:
    SamplingFreq = 150
elif sensingMode == MODE_INDIVIDUAL:
    SamplingFreq = 60
    scale_factor = np.array([6.6158,4.8674,3.8681,4.4218])
    individual_baseline_g = np.array([43165.6724890830,30012.3842794,43506.0567685590,33923.1353711790])
    lin_rot_baseline = np.array([5135.94782608696,4722.31594202899,3894.76563146998,5195.18053830228])
    # lin_rot_baseline_g = np.array([0,0,0,0])
    # scale_factor = np.array([1,1,1,1])
    # individual_baseline_g = np.array([0,0,0,0])
elif sensingMode == MODE_LINEAR_ROTATIONAL:
    SamplingFreq = 600
elif sensingMode == MODE_ONE_AND_FOUR:
    SamplingFreq = 1e3
    num_sensors_2 = 4
    IsTwoModeMerged = True
elif sensingMode == MODE_ONE_AND_NINE:
    SamplingFreq = 1e3
    num_sensors_2 = 9
    IsTwoModeMerged = True
elif sensingMode == MODE_NINE_AND_INDIV:
    SamplingFreq = 110
    num_sensors_2 = 36
    IsTwoModeMerged = True
elif sensingMode == MODE_FOUR_AND_INDIV:
    SamplingFreq = 250
    num_sensors_2 = 36
    IsTwoModeMerged = True
elif sensingMode == MODE_TOR_NS_AND_INDIV:
    SamplingFreq = 250
    num_sensors_2 = 36
    IsTwoModeMerged = True

if sensingMode == MODE_INDIVIDUAL:
    windowBufferTemp = np.zeros([30,36]);
    wbtt = np.zeros([30,12])

#%% 
SamplingPeriod = 1.0/SamplingFreq
KitprogTimerClockFreq = 100e3
tempPeriodInput = divmod(math.floor(SamplingPeriod * KitprogTimerClockFreq - 1) , 2**8)
periodInput = np.array([tempPeriodInput[1], tempPeriodInput[0]])

#%%

if SensorExist:
    import datetime
    currDateString = datetime.datetime.now().strftime("%y%m%d_%H%M%S_")
#    output_file = ResultSavingDirectory + '\\'+ 'result_' +currDateString + SavingFileName + '.html'

    #ts = psoc.TactileSensor(port="COM7")
    #ts.ser.flushInput()
    
    # ts.sendChar("i")
    # time.sleep(0.01)

    thisInputArray = np.array([ord('a'), SensorNum, SensorAddress[0], SensorAddress[1]])    
    #ts.sendNum(thisInputArray)
    time.sleep(0.1)
    
#    fwrite(com,['p' periodInput(1) periodInput(2)])
    thisInputArray = np.array([ord('p'), periodInput[0], periodInput[1]])
    #ts.sendNum(thisInputArray)
    # ts.sendChar("p")    
    # ts.sendNum(periodInput[0])
    # ts.sendNum(periodInput[1])
    time.sleep(0.1)
    
    thisInputArray = np.array([ord('m'), sensingMode])    
    #ts.sendNum(thisInputArray)
    time.sleep(0.1)
    
    #ts.sendChar("q")
    time.sleep(0.1)
    
    
    #ts.packet_size = ord(ts.ser.read(1))-1
    
    #num_sensors_1= (ts.packet_size - 1) / 2
    if sensingMode == MODE_INDIVIDUAL:
        num_sensors_1 = 4
        
    #ts.num_sensors= (ts.packet_size - 1) / 2
    #ts.unpackFormat = '<'
    #for i in range(0,ts.packet_size):
    #    ts.unpackFormat = ts.unpackFormat + 'B'
        
    
    if IsTwoModeMerged: #% Deal with Merging Techniq
        num_sensors_1 = num_sensors_1/2
        
        #sensorIndexInData_1 = list(range(0,(ts.packet_size - 1) / 2, 2) )
        #sensorIndexInData_2 = list(range(1,(ts.packet_size - 1) / 2, 2) )
        
        groupIndexMax = num_sensors_2/ num_sensors_1 -1; #% Follows C convention
        
    else:
        num_sensors_2 = 0   # % Dummy
    
    print("num_sensor 1=")
    num_sensors_1 = int(num_sensors_1)
    print(num_sensors_1)
    print("num_sensor 2=")
    num_sensors_2 = int(num_sensors_2)
    print(num_sensors_2)
    
    
#######################################################


    tic = time.time()
    
    #Start Streaming
    #ts.sendChar("s")
    
    #%% Get Initial samples for measuring Offset Values    
    initialSamplingNum = 16 # it should be an even number
    
    #%% Buffer
    init_BufferSize = 5000;
    sensor_1_data_history_first = np.zeros((init_BufferSize,num_sensors_1))
    sensor_1_data_history_second = np.zeros((init_BufferSize,num_sensors_2))
    sensor_1_offset_first = np.zeros((1, num_sensors_1))
    sensor_1_offset_second = np.zeros((1, num_sensors_2))
    read_count_sns_1_first = 0
    read_count_sns_1_second = 0
    sensor_1_offsetObtained = False

    sensor_2_data_history_first = np.zeros((init_BufferSize,num_sensors_1))
    sensor_2_data_history_second = np.zeros((init_BufferSize,num_sensors_2))
    sensor_2_offset_first = np.zeros((1, num_sensors_1))
    sensor_2_offset_second = np.zeros((1, num_sensors_2))
    sensor_2_offsetObtained = False
    read_count_sns_2_first = 0
    read_count_sns_2_second = 0

    
    # Some initial settings for the FFT.. preliminary here, will move up later
    winSizeT = 500 * 0.001 # in sec
    overlapT = 100e-3

    Fs = SamplingFreq
    T = 1.0 / Fs
    # Window is set to be 100ms
    winSizeN = int(winSizeT / T)
    hammingW = np.hamming(winSizeN)

    overlapN = int(overlapT / T) #sample Count
    isFirst = 1
    
    # Buffer stuff for the window and fft
    waitUntil_sns_1_first = 0
    waitUntil_sns_2_first = 0
    
    fftStoreCounter_sns_1 = 0;
    fftStoreCounter_sns_2 = 0;

   
    fftStorage_sns_1_NS = np.zeros((init_BufferSize, winSizeN//2))
    fftStorage_sns_1_WE = np.zeros((init_BufferSize, winSizeN//2))
    
    fftStorage_sns_2_NS = np.zeros((init_BufferSize, winSizeN//2))
    fftStorage_sns_2_WE = np.zeros((init_BufferSize, winSizeN//2))

    



    #ts.sendChar("i")
    

#%%

tic = time.time()

stopCMDsent = False


snsIndex = 0

history = np.genfromtxt('psocScanner.cpython-38.pyc', delimiter=',', encoding="IBM037")
print(history)
for data in history:
    if SensorExist:
        #Read rest of the data
        tempSampledData = data
        
        if sensingMode == MODE_INDIVIDUAL:
            wbtt = tempSampledData[0][[2, 3, 6, 9, 10, 15, 21, 24, 27, 28, 32, 33]]
            wbtt0 = (wbtt.sum(axis=0) - individual_baseline_g[0]) / scale_factor[0] + lin_rot_baseline[0]
            wbtt = tempSampledData[0][[1,5,9,13,17,21,25,29,33]]
            wbtt1 = (wbtt.sum(axis=0) - individual_baseline_g[1]) / scale_factor[1] + lin_rot_baseline[1]
            wbtt = tempSampledData[0][[0, 1, 4, 8, 11, 13, 23, 25, 26, 30, 34, 35]]
            wbtt2 = (wbtt.sum(axis=0) - individual_baseline_g[2]) / scale_factor[2] + lin_rot_baseline[2]
            wbtt = tempSampledData[0][[3,7,11,15,19,23,27,31,35]]
            wbtt3 = (wbtt.sum(axis=0) - individual_baseline_g[3]) / scale_factor[3] + lin_rot_baseline[3]
            tempSampledData = np.array([[wbtt0,wbtt1,wbtt2,wbtt3]])
        # print(tempSampledData)
        # print(tempSampledData.shape)
        
        if snsIndex == 0:
            if IsTwoModeMerged:
                # First Mode
                sensor_1_data_history_first[read_count_sns_1_first,:] = tempSampledData[0,sensorIndexInData_1] - sensor_1_offset_first
                read_count_sns_1_first += 1

                

                # Second Mode
                groupIndex = ts.groupIndex
                sensor_1_data_history_second[read_count_sns_1_second, groupIndex*num_sensors_1:(groupIndex+1)*num_sensors_1]= tempSampledData[0,sensorIndexInData_2]
                if groupIndex == groupIndexMax:
                    sensor_1_data_history_second[read_count_sns_1_second,:] = sensor_1_data_history_second[read_count_sns_1_second,:] - sensor_1_offset_second
                    read_count_sns_1_second = read_count_sns_1_second+1;

                # Obtain Offset from initial few samples    
                if not sensor_1_offsetObtained and  read_count_sns_1_second == initialSamplingNum:
                    sensor_1_offset_first = np.mean(sensor_1_data_history_first[3:initialSamplingNum,:],axis=0)
                    sensor_1_offset_second = np.mean(sensor_1_data_history_second[3:initialSamplingNum,:],axis=0)
                    sensor_1_offsetObtained = True
                    waitUntil_sns_1_first = winSizeN + read_count_sns_1_first

            else:
                # print(tempSampledData)
                sensor_1_data_history_first[read_count_sns_1_first,:] = tempSampledData - sensor_1_offset_first
                read_count_sns_1_first += 1
                if not sensor_1_offsetObtained and  read_count_sns_1_first == initialSamplingNum:
                    sensor_1_offset_first = np.mean(sensor_1_data_history_first[3:initialSamplingNum,:],axis=0)                    
                    sensor_1_offsetObtained = True
                    waitUntil_sns_1_first = winSizeN + read_count_sns_1_first

            
            #if the data overflows
            if read_count_sns_1_first > sensor_1_data_history_first.shape[0]-1:
                sensor_1_data_history_first = np.append(sensor_1_data_history_first, np.zeros((init_BufferSize,num_sensors_1)), axis=0)
            if read_count_sns_1_second > sensor_1_data_history_second.shape[0]-1:
                sensor_1_data_history_second = np.append(sensor_1_data_history_second, np.zeros((init_BufferSize,num_sensors_2)), axis=0)
            
            if SensorNum == 2:
                snsIndex = 1
                
            #print(sensor_1_data_history_first.shape)
            ######## Runs for 4 Ch case only
            # Calculate FFT
            if read_count_sns_1_first == waitUntil_sns_1_first:
                #process window data. filter out dc and multiply a hamming window
                windowBuffer = sensor_1_data_history_first[waitUntil_sns_1_first - winSizeN  : waitUntil_sns_1_first, :]
                # print(windowBuffer.shape)
                #[s1,s2]=windowBuffer.shape
                #if s2 == 0:
                #    windowBuffer = np.zeros([30,36])
                # if sensingMode == MODE_INDIVIDUAL:
                    # wbtt = windowBuffer[:,[2, 3, 6, 9, 10, 15, 21, 24, 27, 28, 32, 33]]
                    # wbtt0 = (wbtt.sum(axis=1) - individual_baseline_g[0]) / scale_factor[0] + lin_rot_baseline[0]
                    # wbtt = windowBuffer[:,[1,5,9,13,17,21,25,29,33]]
                    # wbtt1 = (wbtt.sum(axis=1) - individual_baseline_g[1]) / scale_factor[1] + lin_rot_baseline[1]
                    # wbtt = windowBuffer[:,[0, 1, 4, 8, 11, 13, 23, 25, 26, 30, 34, 35]]
                    # wbtt2 = (wbtt.sum(axis=1) - individual_baseline_g[2]) / scale_factor[2] + lin_rot_baseline[2]
                    # wbtt = windowBuffer[:,[3,7,11,15,19,23,27,31,35]]
                    # wbtt3 = (wbtt.sum(axis=1) - individual_baseline_g[3]) / scale_factor[3] + lin_rot_baseline[3]
                    # Differential_N_S = wbtt1 - wbtt3
                    # Differential_W_E = wbtt0 - wbtt2 # It could be Torsional
                # else:
                # print(windowBuffer.shape)
                Differential_N_S = windowBuffer[:,1] - windowBuffer[:,3]
                Differential_W_E = windowBuffer[:,0] - windowBuffer[:,2] # It could be Torsional
                # print(Differential_W_E.shape)
                # print(wbtt.sum(axis=1))

                # FFT for N-S
                if slopeCompensateOn:
                    temp = np.transpose(Differential_N_S)
                    # temp = temp[0,:]
                    x_fit = np.array(range(0,np.shape(temp)[0]))
                    
                    fitObject = np.poly1d(np.polyfit(x_fit, temp, 1))

                    fftInput = np.multiply((Differential_N_S - fitObject(x_fit)), hammingW)

                else:
                    fftInput = np.multiply((Differential_N_S - np.mean(Differential_N_S)), hammingW)
                
                yfft = scipy.fftpack.fft(fftInput)
                xfft = np.linspace(0.0, Fs/2, winSizeN//2)

                fftStorage_sns_1_NS[fftStoreCounter_sns_1,:] = np.abs(yfft[:winSizeN//2])

                #FFT for W-E or Torsional
                if slopeCompensateOn:
                    temp = np.transpose(Differential_W_E)
                    # temp = temp[0,:]
                    x_fit = np.array(range(0,np.shape(temp)[0]))
                    
                    fitObject = np.poly1d(np.polyfit(x_fit, temp, 1))

                    fftInput = np.multiply((Differential_W_E - fitObject(x_fit)), hammingW)
                    
                else:
                    fftInput = np.multiply((Differential_W_E - np.mean(Differential_W_E)), hammingW)

                yfft = scipy.fftpack.fft(fftInput)
                xfft = np.linspace(0.0, Fs/2, winSizeN//2)

                fftStorage_sns_1_WE[fftStoreCounter_sns_1,:] = np.abs(yfft[:winSizeN//2])

                yfft_slip = np.abs(yfft[:winSizeN//2])
                count_slip = np.count_nonzero(yfft_slip[0:10]> slip_threshold)
                count_noise = np.count_nonzero(yfft_slip[10:15]> slip_threshold)
                A = np.around(yfft_slip)
                # A = np.astype(int)
                
                print(A[0:15])
                #print('[[[[[[[[[[[',str(max(np.around(yfft_slip[0:10]))),'==',str(max(np.around(yfft_slip[10:15]))),', ',str(count_slip),', ',str(count_noise),']]]]]]]]]]]')
                #print()
                
                #print(count_slip)
                # print(str(int(max(yfft_slip[0:15]))),', ',str(int(min(yfft_slip[0:15]))))
                # print(yfft_slip[0:15].astype(int))
                
                if count_slip > 1:
                    if count_noise <6:
                        CurrentPosition += addClose
                        if CurrentPosition > close_max:
                            CurrentPosition = close_max
                        # r.setPosition(CurrentPosition)
                        print('SLIP')
                        # print(CurrentPosition)
                        
                        winsound.Beep(frequency, duration)
                    else:
                        print('NOISE')
                
                
                #if yfft_slip[0:10] > 200:
                    #print(yfft_slip[0])
                #print(fftStorage_sns_1_WE[fftStoreCounter_sns_1,0:4])

                waitUntil_sns_1_first += overlapN
                fftStoreCounter_sns_1 += 1

                if fftStoreCounter_sns_1 > fftStorage_sns_1_NS.shape[0]-1:
                    fftStorage_sns_1_NS = np.append(fftStorage_sns_1_NS, np.zeros((init_BufferSize,winSizeN//2)), axis=0)
                    fftStorage_sns_1_WE = np.append(fftStorage_sns_1_WE, np.zeros((init_BufferSize,winSizeN//2)), axis=0)
                



        
        elif snsIndex == 1:
            if IsTwoModeMerged:
                sensor_2_data_history_first[read_count_sns_2_first,:] = tempSampledData[0,sensorIndexInData_1]- sensor_2_offset_first
                read_count_sns_2_first += 1

                groupIndex = ts.groupIndex
                sensor_2_data_history_second[read_count_sns_2_second, groupIndex*num_sensors_1:(groupIndex+1)*num_sensors_1]= tempSampledData[0,sensorIndexInData_2]
                if groupIndex == groupIndexMax:
                    sensor_2_data_history_second[read_count_sns_2_second,:] = sensor_2_data_history_second[read_count_sns_2_second,:] - sensor_2_offset_second
                    read_count_sns_2_second = read_count_sns_2_second+1;

                #Obtain offsets
                if not sensor_2_offsetObtained and  read_count_sns_2_second == initialSamplingNum:
                    sensor_2_offset_first = np.mean(sensor_2_data_history_first[3:initialSamplingNum,:],axis=0)                    
                    sensor_2_offset_second = np.mean(sensor_2_data_history_second[3:initialSamplingNum,:],axis=0)                    
                    sensor_2_offsetObtained = True
                    waitUntil_sns_2_first = winSizeN + read_count_sns_2_first
            else:
                sensor_2_data_history_first[read_count_sns_2_first,:] = tempSampledData
                read_count_sns_2_first += 1

                if not sensor_2_offsetObtained and  read_count_sns_2_first == initialSamplingNum:
                    sensor_2_offset_first = np.mean(sensor_2_data_history_first[3:initialSamplingNum,:],axis=0)                    
                    sensor_2_offsetObtained = True
                    waitUntil_sns_2_first = winSizeN + read_count_sns_2_first

            
            #if the data overflows
            if read_count_sns_2_first > sensor_2_data_history_first.shape[0]-1:
                sensor_2_data_history_first = np.append(sensor_2_data_history_first, np.zeros((init_BufferSize,num_sensors_1)), axis=0)
            if read_count_sns_2_second > sensor_2_data_history_second.shape[0]-1:
                sensor_2_data_history_second = np.append(sensor_2_data_history_second, np.zeros((init_BufferSize,num_sensors_2)), axis=0)
            
            snsIndex = 0


            ######## Runs for 4 Ch case only

            # Calculate FFT    
            if read_count_sns_2_first == waitUntil_sns_2_first:
                #process window data. filter out dc and multiply a hamming window
                windowBuffer = sensor_2_data_history_first[waitUntil_sns_2_first - winSizeN  : waitUntil_sns_2_first, :]
                Differential_N_S = windowBuffer[:,1] - windowBuffer[:,3]
                Differential_W_E = windowBuffer[:,0] - windowBuffer[:,2]

                # FFT for N-S
                if slopeCompensateOn:
                    temp = np.transpose(Differential_N_S)

                    # temp = temp[0,:]
                    x_fit = np.array(range(0,np.shape(temp)[0]))
                    
                    fitObject = np.poly1d(np.polyfit(x_fit, temp, 1))

                    fftInput = np.multiply((Differential_N_S - fitObject(x_fit)), hammingW)


                else:
                    fftInput = np.multiply((Differential_N_S - np.mean(Differential_N_S)), hammingW)
                yfft = scipy.fftpack.fft(fftInput)
                xfft = np.linspace(0.0, Fs/2, winSizeN//2)

                fftStorage_sns_2_NS[fftStoreCounter_sns_2,:] = np.abs(yfft[:winSizeN//2])

                #FFT for W-E
                if slopeCompensateOn:
                    temp = np.transpose(Differential_W_E)
                    # temp = temp[0,:]
                    x_fit = np.array(range(0,np.shape(temp)[0]))
                    
                    fitObject = np.poly1d(np.polyfit(x_fit, temp, 1))

                    fftInput = np.multiply((Differential_W_E - fitObject(x_fit)), hammingW)
                    
                else:
                    fftInput = np.multiply((Differential_W_E - np.mean(Differential_W_E)), hammingW)
                
                yfft = scipy.fftpack.fft(fftInput)
                xfft = np.linspace(0.0, Fs/2, winSizeN//2)

                fftStorage_sns_2_WE[fftStoreCounter_sns_2,:] = np.abs(yfft[:winSizeN//2])

                waitUntil_sns_2_first += overlapN
                fftStoreCounter_sns_2 += 1

                if fftStoreCounter_sns_2 > fftStorage_sns_2_NS.shape[0]-1:
                    fftStorage_sns_2_NS = np.append(fftStorage_sns_2_NS, np.zeros((init_BufferSize,winSizeN//2)), axis=0)
                    fftStorage_sns_2_WE = np.append(fftStorage_sns_2_WE, np.zeros((init_BufferSize,winSizeN//2)), axis=0)

if SensorExist:    
    ts.sendChar("i") #End the communication.
    
    sensor_1_data_history_first = sensor_1_data_history_first[0:read_count_sns_1_first-1,:]
    fftStorage_sns_1_NS = fftStorage_sns_1_NS[0:fftStoreCounter_sns_1,:]
    fftStorage_sns_1_WE = fftStorage_sns_1_WE[0:fftStoreCounter_sns_1,:]
    
    if IsTwoModeMerged:
        sensor_1_data_history_second = sensor_1_data_history_second[0:read_count_sns_1_second-1,:]

    if SensorNum == 2:
        sensor_2_data_history_first = sensor_2_data_history_first[0:read_count_sns_2_first-1,:]
        fftStorage_sns_2_NS = fftStorage_sns_2_NS[0:fftStoreCounter_sns_2,:]
        fftStorage_sns_2_WE = fftStorage_sns_2_WE[0:fftStoreCounter_sns_2,:]

        if IsTwoModeMerged:
            sensor_2_data_history_second = sensor_2_data_history_second[0:read_count_sns_2_second-1,:]

    

#r.stopMotion()

# print(fftStorage_sns_1_WE.shape)
if SensorExist:
    ts.closePort()
    
    # DO TORSIONAL SLIP STUFF
    #print(sensor_1_data_history_first.shape)
    
    #r.demo()
    
    if plotShow:
        #%% Plot the data
        import matplotlib.pyplot as plt
        # plt.figure()
        # plt.plot(sensor_2_data_history_first)
        # plt.ylabel('Digital Count')
        # plt.xlabel('sample')
        # plt.grid()
        # plt.show()

        # plt.figure()
        # plt.plot(sensor_2_data_history_second)
        # plt.ylabel('Digital Count')
        # plt.xlabel('sample')
        # plt.grid()
        # plt.show()

        plt.figure()
        plt.plot(fftStorage_sns_1_NS[:,0:5])
        plt.ylabel('| FFT |')
        plt.xlabel('sample')
        plt.grid()
        plt.show()
        
        plt.figure()
        plt.plot(fftStorage_sns_1_WE[:,0:5])
        plt.ylabel('| FFT |')
        plt.xlabel('sample')
        plt.grid()
        plt.show()
        
        plt.figure()
        plt.plot(sensor_1_data_history_first[16:-1,:])
        plt.ylabel('| FFT |')
        plt.xlabel('sample')
        plt.grid()
        plt.show()
        
        #plt.figure()
        #plt.plot(fftStorage_sns_2_NS[:,0:5])
        #plt.ylabel('Digital Count')
        #plt.xlabel('sample')
        #plt.grid()
        #plt.show()
    
    # #%% Save Output
    # import datetime
    # currDateOnlyString = datetime.datetime.now().strftime("%y%m%d")
    # directory = ResultSavingDirectory +'/' + currDateOnlyString

    # if not os.path.exists(directory):
    #     os.makedirs(directory)

    # currDateTimeString = datetime.datetime.now().strftime("%y%m%d_%H%M%S_")


    # output_file = directory + '/'+ 'result_' +currDateTimeString + SavingFileName + '.csv'
    
    # np.savetxt(output_file, sensor_1_data_history_first, delimiter=",")
    
    CurrentPosition = 100
    # r.setPosition(CurrentPosition)