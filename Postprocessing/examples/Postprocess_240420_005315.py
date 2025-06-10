# -*- coding: utf-8 -*-
"""
Created on Sun Apr 16 09:29:37 2023

@author: Laktop
"""
import sys

import numpy as np
import matplotlib.pyplot as pl
from matplotlib.gridspec import GridSpec
import pandas as pd
import glob
from cmcrameri import cm as cmCrameri
import time

scriptspath= 'C:\..\...'
pcpath= 'C:\..\...'

sys.path.append(scriptspath+r'\pyLEM')

from dataADC import*


# %%  Settings

filename='240420_005315'
date=r'\20240419'


path=pcpath+ r'\2024_Fieldwork\LEM\data'+date
fileLASER=path+r'\INS'+filename+'.csv'
fileADC=path+r'\ADC'+filename+'.csv'
savpath=pcpath+ r'\2024_Fieldwork\LEM\plots'

SPS=19200
window=1920
flowpass=15
INSkargs={}

MultiFreq=True
i_autoCal=0
dT_start=-3.425

i_blok=[int(50*SPS),int(55*SPS)]

# data margins
start=35
stop=390

# climbs
t_str=[115]
t_stp=[147.5]
h_tot=[1.45]
w_depth=[2.30]


# emagpy params
w_cond=2408
d_coils=2.027




# %% plot section of raw data

t0=0 # start time
t1=10  # stop time

lookupSectionRaw(fileADC,t0,t1,SPS=19200,units='seconds',title='File:'+filename, channels=[1,2])

# %% plot INS data
dataINS=INSLASERdata(fileLASER,correct_Laser=True,
                     roll0=0.0,pitch0=0.0)

plot_summary(dataINS,getextent(dataINS),heading=False)



# %% Load, merge and save data ADC +INS+ Laser
datamean,dataINS,params=processDataLEM(path,filename,plot=True,savefile=True,
                          window=window,flowpass=flowpass,chunksize=38401,
                          Tx_ch='ch2', Rx_ch=['ch1'],
                          findFreq=True,i_blok=i_blok,
                          MultiFreq=False,
                          INSkargs=INSkargs,
                          autoCal=True,i_autoCal=i_autoCal,dT_start=dT_start)
# datamean['time']=datamean.TOW-datamean.TOW[0]






# %% Load processed data
[datamean,dataINS,params]=sl.load_pkl(path+r'\LEM'+filename+'.pkl')




# %% cut data 

datamean=trim_data(start,stop,datamean,params)

# %%% plots of I,Q and h

plot_QIandH(datamean,params,title='')


# %%% calibration climbs

data_climbs=Fit_climbs(datamean,params,
               t_str,t_stp, h_tot,w_depth=w_depth,shallow=True,
               w_cond=w_cond,d_coils=d_coils,
               plot=True)



# %% invert data

Invert_data(datamean,params,
               w_cond=2408,d_coils=0,
               plot=True)



# %% plot inverted data


fig,ax=pl.subplots(1,1,sharex=True)
ax.plot(datamean.time,datamean.hw_invQ,'x',label='EM Q')
ax.plot(datamean.time,datamean.hw_invI,'x',label='EM I')
ax.plot(datamean.time,datamean.hw_invQI,'x',label='EM Q+I')
ax.plot(datamean.time,datamean.h_GPS,'--k',label='GPS')
ax.plot(datamean.time,datamean.h_Laser,'--b',label='Laser')
pl.xlabel('time (s)')
ax.set_ylabel('h (m)')
ax.legend()



fig,ax=pl.subplots(1,1,sharex=True)
ax.plot(datamean.h_Laser,datamean.hw_invQ,'x',label='Laser all')
ax.plot(datamean.h_GPS,datamean.hw_invQ,'x',label='GPS deep water')
ax.plot([0,30],[0,30],'--k')
ax.set_ylabel('h EM (m)')
ax.set_xlabel('h Laser/GPS (m)')
ax.legend()
ax.set_xlim(0,25)
ax.set_ylim(0,25)



fig,ax=pl.subplots(1,1,sharex=True)
ax.plot(datamean.h_Laser,datamean.hw_invQ,'x',label='EM Q')
ax.plot(datamean.h_Laser,datamean.hw_invI,'x',label='EM I')
ax.plot(datamean.h_Laser,datamean.hw_invQI,'x',label='EM Q+I')
ax.plot([0,30],[0,30],'--k')
ax.set_ylabel('h EM (m)')
ax.set_xlabel('h Laser (m)')
ax.legend()
# ax.set_xlim(0,25)
# ax.set_ylim(0,25)


fig,ax=pl.subplots(1,1,sharex=True)
ax.plot(datamean.time,datamean.h_totQ,'x',label='EM Q')
ax.plot(datamean.time,datamean.h_totI,'x',label='EM I')
ax.plot(datamean.time,datamean.h_totQI,'x',label='EM Q+I')
pl.xlabel('time (s)')
ax.set_ylabel('Total Thickness (ice+snow) (m)')
ax.legend()



# %% save inverted data

saveDataLEM(datamean, dataINS,params)


