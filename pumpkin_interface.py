import serial
import time
import tkinter as tk
import sys
import tkinter.font

SERIALPORT='COM7'

def on_closing():
    #raise SystemExit
    print("exit")
    window.destroy()

def fmat(tgt):
    tgt.configure(font=('Helvetica', int(textSize), 'bold'), bg=bgnd, fg=fgnd, highlightbackground='#ddd', highlightthickness=1)

try:
    pumpkuter = serial.Serial(port=SERIALPORT, baudrate=115200, timeout=.1)
except serial.SerialException:
    #print("Serial port not connected!")
    sys.exit("Serial port not connected!")


#fnt = "Comic Sans MS"
bgnd = '#026'
fgnd = 'white'
vpad = 5
hpad = 15
window_open=True

degree_sign = u'\N{DEGREE SIGN}'

root = tk.Tk()
root.title("Pumpkuter v3.1")
window = tk.Frame(master=root)
window.pack(side="top", fill="both", expand=True)
#window.configure(bg='green2')
window.configure(bg='#026')
lbl_test=tk.Label(master=window, text='asdf')
lbl_test.grid(row=0, column=0)


for i in range(5):
    window.columnconfigure(i, weight=1, minsize=20)
    
window.rowconfigure(0,weight=0, minsize=50)

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
lbl_hdg.grid(row=1, column=2, sticky='nsew')
lbl_bng = tk.Label(master=window, padx=hpad, pady=vpad)
lbl_bng.grid(row=1, column=1, sticky='nsew')


states=['AOS','RUN']
state_colors=['red','green']

while window_open:
    textSize = window.winfo_width()/41
    fnt = ('Arial', int(textSize))
    #print(textSize)
    line = pumpkuter.readline().decode('utf-8')[:-2]
    if(line):
        telem = line.split(',')
        hours = (int(telem[0]) + 18)%24
        minutes = int(telem[1])
        seconds = int(telem[2])
        time = '{:02}:{:02}:{:02} CST'.format(hours,minutes,seconds)
        state = int(telem[3])
        fix = int(telem[4])
        sats = int(telem[5])
        lat = float(telem[6])
        lon = float(telem[7])
        alt = int(telem[8])
        hdg = int(telem[9])
        vel = float(telem[10])
        err = int(telem[11])
        time_to_drop = int(telem[12])
        bearing = int(telem[13])
        
        
        
        lbl_time.configure(text=time)
        lbl_state.configure(text='State: '+states[state], bg=state_colors[state], font=fnt)
        lbl_latitude.configure(text='Lat:{:08.5f}{:s}'.format(lat,degree_sign))
        lbl_longitude.configure(text='Lon:{:08.5f}{:s}'.format(lon,degree_sign))
        lbl_sats.configure(text='Sats:{:d}'.format(sats))
        lbl_alt.configure(text='Alt:{:4d}ft'.format(alt))
        lbl_hdg.configure(text='hdg {:03d}'.format(hdg))
        lbl_vel.configure(text='V:{:4.1f}mph'.format(vel))
        lbl_err.configure(text='err:{:3d}'.format(err))
        lbl_ttd.configure(text='t:{:2d}:{:02d}'.format(time_to_drop//60,time_to_drop%60))
        lbl_bng.configure(text='path: {:03d}'.format(bearing))
        
        fmat(lbl_time)
        #fmat(lbl_state)
        fmat(lbl_sats)
        fmat(lbl_latitude)
        fmat(lbl_longitude)
        fmat(lbl_alt)
        fmat(lbl_hdg)
        fmat(lbl_vel)
        fmat(lbl_err)
        fmat(lbl_ttd)
        fmat(lbl_bng)
        
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.update_idletasks()
    root.update()

