import serial
import time
import tkinter as tk
import sys
import tkinter.font
from PIL import Image, ImageTk
import logging

logging.getLogger('root').setLevel('INFO')

SERIALPORT='COM7'
feet_per_division = 3

def on_closing():
    #raise SystemExit
    print("exit")
    window.destroy()

def fmat(tgt):
    tgt.configure(font=('Helvetica', int(textSize), 'bold'), bg=bgnd, fg=fgnd, highlightbackground='#ddd', highlightthickness=1)

ttd_precise = False

#fnt = "Comic Sans MS"
bgnd = '#026'
fgnd = 'white'
vpad = 5
hpad = 15
window_open=True

degree_sign = u'\N{DEGREE SIGN}'

last_state = 0

root = tk.Tk()
root.title("Pumpkuter v3.1")
window = tk.Frame(master=root)
window.pack(side="top", fill="x", expand=False)
#window.configure(bg='green2')
window.configure(bg='#026')

frm_bar = tk.Frame(master=root)
frm_bar.pack(side="top", fill="both", expand=True)
frm_bar.configure(bg='green2')

lbl_test=tk.Label(master=window, text='No Serial Connection on '+SERIALPORT, font=('Helvetica', 50), bg='red')
lbl_test.grid(row=0, column=0)

connection=False
while(not connection):
    try:
        pumpkuter = serial.Serial(port=SERIALPORT, baudrate=115200, timeout=.1)
        connection = True
        lbl_test.destroy()
        logging.info("Connected to Pumpkuter")
    except serial.SerialException:
        root.update_idletasks()
        root.update()
        pass
        #print("Serial port not connected!")


for i in range(5):
    window.columnconfigure(i, weight=1, minsize=20)
    
window.rowconfigure(0,weight=0, minsize=30)

lbl_time = tk.Label(master=window, text='', padx=hpad, pady=vpad)
lbl_time.grid(row=0, column=0, sticky='nsew')
lbl_state = tk.Label(master=window, padx=hpad, pady=vpad, highlightbackground='#111', highlightthickness=2)
lbl_state.grid(row = 1, column=0, sticky='nsew')
lbl_latitude = tk.Label(master=window, padx=hpad, pady=vpad)
lbl_latitude.grid(row = 0, column=1, sticky='nsew')
lbl_longitude = tk.Label(master=window, padx=hpad, pady=vpad)
lbl_longitude.grid(row = 0, column=2, sticky='nsew')
lbl_sats = tk.Label(master=window, padx=hpad, pady=vpad)
lbl_sats.grid(row=2, column=0, sticky='nsew')
lbl_alt = tk.Label(master=window, padx=hpad, pady=vpad)
lbl_alt.grid(row=0, column=3, sticky='nsew')
lbl_vel = tk.Label(master=window, padx=hpad, pady=vpad)
lbl_vel.grid(row=0, column=4, sticky='nsew')
lbl_err = tk.Label(master=window, padx=hpad, pady=vpad)
lbl_err.grid(row=1, column=4, sticky='nsew')
lbl_ttd = tk.Label(master=window, padx=hpad, pady=vpad)
lbl_ttd.grid(row=2, column=4, sticky='nsew')
lbl_hdg = tk.Label(master=window, padx=hpad, pady=vpad)
lbl_hdg.grid(row=2, column=1, sticky='nsew')
lbl_bng = tk.Label(master=window, padx=hpad, pady=vpad)
lbl_bng.grid(row=1, column=1, sticky='nsew')
lbl_dist = tk.Label(master=window, padx=hpad, pady=vpad)
lbl_dist.grid(row=1, column=2, sticky='nsew')

can_bar = tk.Canvas(master=frm_bar)
can_bar.pack(fill='both', expand=True)
track = can_bar.create_line(100,200,200,50, fill='#999', width=6)
can_bar.configure(bg='blue')
plane_marker = can_bar.create_text(20, 20, text="^")


states=['AOS','RUN','TERM','SDWN', 'ABORT', 'TGT']
state_colors=['red','green','yellow','blue', 'orange', 'blue']

while window_open:
    textSize = window.winfo_width()/42
    fnt = ('Arial', int(textSize))
    #print(textSize)
    line = pumpkuter.readline().decode('utf-8')[:-2]
    
    if(line):
        
        if('TIMR' in line):
            time_to_drop=float(line.split(',')[1])
            ttd_precise=True
        
        if('DROP' in line):
            lbl_ttd.configure(bg='green')
            root.update()
            
            telem = line.split(',')
            hours = (int(telem[1]) + 18)%24
            minutes = int(telem[2])
            seconds = int(telem[3])
            time = '{:02}:{:02}:{:02} CST'.format(hours,minutes,seconds)
            state = int(telem[4])
            fix = int(telem[5])
            sats = int(telem[6])
            lat = float(telem[7])
            lon = float(telem[8])
            alt = int(telem[9])
            hdg = int(telem[10])
            vel = float(telem[11])
            err = int(telem[12])
            time_to_drop = int(telem[13])
            bearing = int(telem[14])
            plane_x_error = float(telem[15])
            plane_y_error = float(telem[16])
            final_gnd_spd = float(telem[17])
            final_drop_height = float(telem[18])
            
            
            drop_log_name='dropLogs/drop{:02}{:02}{:02}.txt'.format(hours,minutes,seconds)
            with open(drop_log_name, 'w') as f:
                f.write('Time: '+time)
                f.write('\nLat:{:08.5f}{:s}'.format(lat,degree_sign))
                f.write('\nLon:{:08.5f}{:s}'.format(lon,degree_sign))
                f.write('\nAlt:{:4d}ft'.format(alt))
                f.write('\nhdg {:03d}'.format(hdg))
                f.write('\nV:{:5.1f}mph'.format(vel))
                f.write('\nPath: {:03d}'.format(bearing))
                f.write('\nPlaneX-dropX: {:05.3f}m'.format(plane_x_error))
                f.write('\nPlaneY-dropY: {:05.3f}m'.format(plane_y_error))
                f.write('\nfinal_ground_speed: {:05.2f}mph'.format(final_gnd_spd))
                f.write('\nfinal_drop_height: {:04.1f}ft'.format(final_drop_height))
        
        if('FDAT' in line):
            telem = line.split(',')
            hours = (int(telem[1]) + 18)%24
            minutes = int(telem[2])
            seconds = int(telem[3])
            time = '{:02}:{:02}:{:02} CST'.format(hours,minutes,seconds)
            state = int(telem[4])
            fix = int(telem[5])
            sats = int(telem[6])
            lat = float(telem[7])
            lon = float(telem[8])
            alt = int(telem[9])
            hdg = int(telem[10])
            vel = float(telem[11])
            err = int(telem[12])
            if(not ttd_precise): time_to_drop = int(telem[13])
            bearing = int(telem[14])
            drop_dist = int(telem[15])
            
        time_bg = 'red'
        if time_to_drop <= 10:
            time_bg = 'yellow'
            if time_to_drop < 1:
                time_bg = 'green'
        
        err_scaled = err / feet_per_division
        err_pixels = err_scaled * can_bar.winfo_width()/15
        
        if(not state==last_state):
            logging.info("state changed to "+states[state])
            last_state=state
        
        lbl_time.configure(text=time)
        lbl_state.configure(text='State: '+states[state], bg=state_colors[state], font=fnt)
        lbl_latitude.configure(text='Lat:{:08.5f}{:s}'.format(lat,degree_sign))
        lbl_longitude.configure(text='Lon:{:08.5f}{:s}'.format(lon,degree_sign))
        lbl_sats.configure(text='Sats:{:d}'.format(sats))
        lbl_alt.configure(text='RAlt:{:4d}ft'.format(alt))
        lbl_hdg.configure(text='hdg {:03d}'.format(hdg))
        lbl_vel.configure(text='V:{:5.1f}mph'.format(vel))
        lbl_err.configure(text='err:{:4d}ft'.format(err))
        lbl_ttd.configure(text='t:{:2d}:{:05.2f}'.format(int(time_to_drop//60),time_to_drop%60), bg=time_bg)
        lbl_bng.configure(text='Path: {:03d}'.format(bearing))
        lbl_dist.configure(text='Range: {:4d} m'.format(drop_dist))
        
        fmat(lbl_time)
        #fmat(lbl_state)
        fmat(lbl_sats)
        fmat(lbl_latitude)
        fmat(lbl_longitude)
        fmat(lbl_alt)
        fmat(lbl_hdg)
        fmat(lbl_vel)
        fmat(lbl_err)
        fmat(lbl_bng)
        fmat(lbl_dist)
        
        lbl_ttd.configure(font=('Helvetica', int(textSize), 'bold'), bg=time_bg, fg='#111', highlightbackground='#ddd', highlightthickness=1)
        
        if(err == 0): bar_bgnd = 'blue'
        if(err > 0): bar_bgnd = 'green'
        if(err < 0): bar_bgnd = 'red'
        
        can_bar.coords(track, can_bar.winfo_width()/2, 10, can_bar.winfo_width()/2, can_bar.winfo_height()-10)

        if(err_scaled<-7):
            can_bar.itemconfig(plane_marker, text="<")
            marker_box = can_bar.bbox(plane_marker)
            plane_marker_x = (marker_box[2]-marker_box[0])*2/4
            can_bar.coords(plane_marker, plane_marker_x, can_bar.winfo_height()/2)
            
        if(err_scaled>7):
            can_bar.itemconfig(plane_marker, text=">")
            marker_box = can_bar.bbox(plane_marker)
            plane_marker_x = can_bar.winfo_width()-(marker_box[2]-marker_box[0])*2/4
            can_bar.coords(plane_marker, plane_marker_x, can_bar.winfo_height()/2)
            
        if(err_scaled>-8 and err_scaled<8):
            can_bar.coords(plane_marker, err_pixels+can_bar.winfo_width()/2, can_bar.winfo_height()/2)
            can_bar.itemconfig(plane_marker, text='^')
        
        can_bar.itemconfig(plane_marker, font=('Helvetica', int(textSize*4), 'bold'), fill=fgnd)
        can_bar.configure(bg=bar_bgnd)
    
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.update_idletasks()
    root.update()

