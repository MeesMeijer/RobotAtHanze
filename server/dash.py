import wx
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg as FigureCanvas
from threading import *
import mapping

import serial

EVT_RESULT_ID = wx.NewIdRef()
ROBOT_DATA_PREFIX, DASH_DATA_PREFIX = ">", "<"

def EVT_RESULT(win, func):
    """Define Result Event."""
    win.Connect(-1, -1, EVT_RESULT_ID, func)


class RobotDataEvent(wx.PyEvent):

    def __init__(self, error: bool, data: str):
        wx.PyEvent.__init__(self)
        self.SetEventType(EVT_RESULT_ID)

        self.error = error
        self.data = data

        # if not error and data.startswith(ROBOT_DATA_PREFIX):
        #     print("proceed with the data")
        #     self.currentState = ""
        #     self.currentPos = ""
        #     self.currentHeading = ""
        #
        #     self.nextState = ""
        #     self.nextPos = ""
        #     self.nextHeading = ""


class WorkerThread(Thread):

    def __init__(self, frame: wx.Frame, port: str):
        Thread.__init__(self)
        self.frame = frame
        self.port = port
        self._want_abort = False
        self.dead = False
        self.dataTosend: list[str] = []
        self.start()

    def run(self):
        try:
            serialConnection = serial.Serial(self.port, 115200)

            while True: 
                if serialConnection.in_waiting > 0: 
                    recvData = serialConnection.readline().decode().strip()
                    print("[debug] - Got data from port:", self.port ,"\n -->", recvData)
                    wx.PostEvent(self.frame, RobotDataEvent(False, recvData))

                if len(self.dataTosend) > 0: 
                    for _ in range(len(self.dataTosend)): 
                        msg = self.dataTosend.pop(0)
                        print(f"[debug] - Sending {len(msg)} bytes. \n -->", msg)
                        serialConnection.write(DASH_DATA_PREFIX.encode() + msg.encode() + b"\n")
                    
                if self._want_abort: 
                    print("[debug] - Abort is requested.")
                    serialConnection.close()
                    self.dead = True
                    return wx.PostEvent(self.frame, RobotDataEvent(True, "Closed due to aborting"))
        
        except Exception as e: 
            print("ERROR FROM WORKER.RUN", e)
            wx.PostEvent(self.frame, RobotDataEvent(True, "Closed due to error. "))
            self.dead = True

    def abort(self):
        self._want_abort = True

class RobotUpdateDialog(wx.Dialog):

    choices = ["X", "Y", "Z", "AA"]
    
    def __init__(self, lastData: dict[str, any]):
        super(RobotUpdateDialog, self).__init__(None, title="Robot Control Dialog", size=(1100, 800))
        # super().__init__(None, title="Robot Control Dialog", size=(300, 250))
        self.lastData = lastData
        self.InitUI()

    def InitUI(self):
        panel = wx.Panel(self)

        sizer = wx.BoxSizer(wx.VERTICAL)

        # --
        posSizer = wx.BoxSizer(wx.HORIZONTAL)

        startPosLabel = wx.StaticText(panel, label="Start Position: (A-Z)")
        self.startPosInput = wx.TextCtrl(panel, value="I-H")

        posSizer.Add(startPosLabel, 0, wx.ALL | wx.LEFT, 5)
        posSizer.Add(self.startPosInput, 0, wx.ALL | wx.EXPAND,1)

        sizer.Add(posSizer, 0, wx.ALL | wx.EXPAND, 10)
        # --

        # --
        blockedPathsSizer = wx.BoxSizer(wx.HORIZONTAL)

        blockedPathsLabel = wx.StaticText(panel, label="Blocked Paths: (A-Z,..)")

        label = "N/A"
        if len(self.lastData["BLOCKED_PATHS"]) > 0:
            label = ",".join(self.lastData["BLOCKED_PATHS"])

        self.blockedPathsInput = wx.StaticText(panel, label=label)

        blockedPathsSizer.Add(blockedPathsLabel, 0, wx.ALL, 5)
        blockedPathsSizer.Add(self.blockedPathsInput, 1, wx.ALL | wx.EXPAND, 5)

        sizer.Add(blockedPathsSizer, 0, wx.ALL | wx.EXPAND, 10)
        # --

        # --
        boxEndpointsSizerV = wx.BoxSizer(wx.VERTICAL)

        boxEndpointsLabel = wx.StaticText(panel, label="Boxes Endpoints")
        boxEndpointsLabelSizer = wx.BoxSizer(wx.HORIZONTAL)

        boxEndpointsLabelSizer.Add(boxEndpointsLabel, 0, wx.ALL, 5)
        boxEndpointsSizerV.Add(boxEndpointsLabelSizer, 0, wx.ALL, 0)

        # +
        blackBoxPosLabel = wx.StaticText(panel, label="Black Box: ")
        self.blackBoxPosInput = wx.Choice(panel, choices=self.choices)
        self.blackBoxPosInput.SetSelection(self.choices.index(self.lastData["BOX_ENDPOINTS"]["BLACK"]))
        blackBoxSizer = wx.BoxSizer(wx.HORIZONTAL)

        blackBoxSizer.Add(blackBoxPosLabel, 0, wx.ALL, 5)
        blackBoxSizer.Add(self.blackBoxPosInput, 1, wx.ALL | wx.EXPAND, 1)

        boxEndpointsSizerV.Add(blackBoxSizer, 0, wx.ALL, 10)
        # -

        # +
        redBoxPosLabel = wx.StaticText(panel, label="Red Box: ")
        self.redBoxPosInput = wx.Choice(panel, choices=self.choices)
        self.redBoxPosInput.SetSelection(self.choices.index(self.lastData["BOX_ENDPOINTS"]["RED"]))
        redBoxSizer = wx.BoxSizer(wx.HORIZONTAL)

        redBoxSizer.Add(redBoxPosLabel, 0, wx.ALL, 5)
        redBoxSizer.Add(self.redBoxPosInput, 1, wx.ALL | wx.EXPAND, 1)

        boxEndpointsSizerV.Add(redBoxSizer, 0, wx.ALL, 10)
        # -

        # +
        greenBoxPosLabel = wx.StaticText(panel, label="Green Box: ")
        self.greenBoxPosInput = wx.Choice(panel, choices=self.choices)
        self.greenBoxPosInput.SetSelection(self.choices.index(self.lastData["BOX_ENDPOINTS"]["GREEN"]))
        greenBoxSizer = wx.BoxSizer(wx.HORIZONTAL)

        greenBoxSizer.Add(greenBoxPosLabel, 0, wx.ALL, 5)
        greenBoxSizer.Add(self.greenBoxPosInput, 1, wx.ALL | wx.EXPAND, 1)

        boxEndpointsSizerV.Add(greenBoxSizer, 0, wx.ALL, 10)
        # -

        # +
        blueBoxPosLabel = wx.StaticText(panel, label="Blue Box: ")
        self.blueBoxPosInput = wx.Choice(panel, choices=self.choices)
        self.blueBoxPosInput.SetSelection(self.choices.index(self.lastData["BOX_ENDPOINTS"]["BLUE"]))
        blueBoxSizer = wx.BoxSizer(wx.HORIZONTAL)

        blueBoxSizer.Add(blueBoxPosLabel, 0, wx.ALL, 5)
        blueBoxSizer.Add(self.blueBoxPosInput, 1, wx.ALL | wx.EXPAND, 1)

        boxEndpointsSizerV.Add(blueBoxSizer, 0, wx.ALL | wx.EXPAND, 10)
        # -

        sizer.Add(boxEndpointsSizerV, 0, wx.ALL, 10)
        # --

        # --
        updateBtnLabel = wx.StaticText(panel, label="Select to Update: ")
        self.updateBtn = wx.CheckBox(panel)
        updateBtnSizer = wx.BoxSizer(wx.HORIZONTAL)

        updateBtnSizer.Add(updateBtnLabel, 0, wx.ALL, 5)
        updateBtnSizer.Add(self.updateBtn, 1, wx.ALL | wx.EXPAND, 1)

        sizer.Add(updateBtnSizer, 0, wx.ALL, 10)
        # --


        # --
        resetBtnLabel = wx.StaticText(panel, label="Select to Reset: ")

        self.resetBtn = wx.CheckBox(panel)
        resetBtnSizer = wx.BoxSizer(wx.HORIZONTAL)

        resetBtnSizer.Add(resetBtnLabel, 0, wx.ALL, 5)
        resetBtnSizer.Add(self.resetBtn, 1, wx.ALL | wx.EXPAND, 1)

        sizer.Add(resetBtnSizer, 0, wx.ALL, 10)
        # --

        panel.SetSizer(sizer)
        sizer.Fit(panel)

        # self.Centre()
        self.Show(True)
        # self.ShowModal()

    def GetData(self):
        if self.updateBtn.GetValue():

            return [
                self.startPosInput.GetValue(),
                self.choices[self.blackBoxPosInput.GetSelection()],
                self.choices[self.redBoxPosInput.GetSelection()],
                self.choices[self.greenBoxPosInput.GetSelection()],
                self.choices[self.blueBoxPosInput.GetSelection()],
            ]

        return False




class RealTimePlot(wx.Frame):
    
    worker: WorkerThread | None = None

    def __init__(self):
        super(RealTimePlot, self).__init__(None, title="Plotter", size=(1100, 800))
        
        self.InitUI()
        self.InitPlot() 
    
    def StartSerialWorker(self, port: str = "COM3"):
        if not self.worker or self.worker.dead:
            print("[debug] - Starting worker thread with port: ", port) 
            self.worker = WorkerThread(self, port)
            return  
        
        print("[debug] - Got SerialWorker already working..")
    
    def StopSerialWorker(self):
        if not self.worker or self.worker.dead: return False
        print("[debug] - Stopping worker thread.")
        self.worker.abort()
        self.worker = None

    def OnBtnSubmit(self, event: wx.CommandEvent):
        btnEvent: wx.Button = event.EventObject
        
        eventName = btnEvent.GetName()
        print("[debug] - Got Btnsubmit event: ", eventName)

        if eventName == "CONNECT":
            self.StartSerialWorker(str(self.robotPort.GetValue()).strip())
            self.robotPort.SetEditable(False)
            self.robotPortConnect.SetName("DISCONNECT")
            self.robotPortConnect.SetLabel("Disconnect")
            #
            # if self.worker and not self.worker.dead:
            #     self.worker.dataTosend.append("testing")

        elif eventName == "DISCONNECT":
            self.StopSerialWorker()
            self.robotPort.SetEditable(True)
            self.robotPortConnect.SetName("CONNECT")
            self.robotPortConnect.SetLabel("Connect")

        elif eventName == "START_ROBOT":
            if self.worker and not self.worker.dead:
                self.worker.dataTosend.append("START_ROBOT")
            else: 
                print("[error] - Trying to send commands, but not connected..")
                
        elif eventName == "STOP_ROBOT":
            if self.worker and not self.worker.dead:
                self.worker.dataTosend.append("STOP_ROBOT")
            else:
                print("[error] - Trying to send commands, but not connected..")

        elif eventName == "UPDATE_ROBOT":
            dialog = RobotUpdateDialog({
                "BLOCKED_PATHS": ["Q-N"],
                "START_POS": "Q-S",
                "BOX_ENDPOINTS": {
                    "BLACK": "X", "RED": "Y", "GREEN": "Z", "BLUE": "X"
                }
            })
            dialog.ShowModal()

            data = dialog.GetData()
            print(data)
            #TODO: Update the robot


            dialog.Destroy()


    def ResetUI():
        pass 

    def ResetPlot():
        pass 



    def InitUI(self):
        panel = wx.Panel(self)
        
        h1 = wx.BoxSizer(wx.VERTICAL)
        h2 = wx.BoxSizer(wx.HORIZONTAL)

        # Everything in V1
        v1 = wx.BoxSizer(wx.VERTICAL)
        
        self.fig, self.ax = plt.subplots()
        self.canvas = FigureCanvas(panel, -1, self.fig)
        
        v1.Add(self.canvas, 1, wx.EXPAND | wx.ALL, 10)

        # Evertthin in V2             
        v2 = wx.BoxSizer(wx.VERTICAL)

        robotInputSizer = wx.BoxSizer(wx.HORIZONTAL)
        
        robotPortLabel = wx.StaticText(panel, label="Serial Port: ")
        self.robotPort = wx.TextCtrl(panel, value="COM3")
        self.robotPortConnect = wx.Button(panel, wx.ID_ANY, label='Connect', name="CONNECT")

        robotInputSizer.Add(robotPortLabel, 0, wx.ALL, 5)
        robotInputSizer.Add(self.robotPort, 0, wx.ALL, 5)
        robotInputSizer.Add(self.robotPortConnect, 1, wx.ALL, 5)

        v2.Add(robotInputSizer, 0, wx.ALL | wx.EXPAND, 5)

        ctrlabel = wx.StaticText(panel, label="Robot Controls:")

        ctrlabelSizer = wx.BoxSizer(wx.HORIZONTAL)

        robotStartBtn = wx.Button(panel, wx.ID_ANY, label="Start", name="start_robot")
        robotStopBtn = wx.Button(panel, wx.ID_ANY, label="Stop", name="stop_robot")
        robotUpdateBtn = wx.Button(panel, wx.ID_ANY, label='Update', name="UPDATE_ROBOT")

        ctrlabelSizer.Add(ctrlabel, 1, wx.ALL | wx.CENTER, 1)
        ctrlabelSizer.Add(robotStartBtn, 0, wx.ALL, 5)
        ctrlabelSizer.Add(robotStopBtn, 0, wx.ALL, 5)
        ctrlabelSizer.Add(robotUpdateBtn, 0, wx.ALL, 5)

        v2.Add(ctrlabelSizer, 0, wx.ALL | wx.Center, 5)

        # robotControlsSizer = wx.BoxSizer(wx.VERTICAL)

        # robotStartStopSizer = wx.BoxSizer(wx.HORIZONTAL)
        
        # robotControlsSizer.Add(robotStartStopSizer, 0, wx.ALL, 5)

        # v2.Add(robotControlsSizer, 0, wx.ALL , 5)

        v2ab = wx.BoxSizer(wx.HORIZONTAL)
        v2a = wx.BoxSizer(wx.VERTICAL)
        
        currentLabel = wx.StaticText(panel, wx.ID_ANY, 'Current')

        bmp = wx.ArtProvider.GetBitmap(wx.ART_TIP, wx.ART_OTHER, (16, 16))
        currentStateIco = wx.StaticBitmap(panel, wx.ID_ANY, bmp)
        currentStateLabel = wx.StaticText(panel, wx.ID_ANY, 'State: ')
        self.currentState = wx.StaticText(panel, wx.ID_ANY, label='N/A')

        bmp = wx.ArtProvider.GetBitmap(wx.ART_TIP, wx.ART_OTHER, (16, 16))
        currentPosIco = wx.StaticBitmap(panel, wx.ID_ANY, bmp)
        currentPosLabel = wx.StaticText(panel, wx.ID_ANY, 'Position: ')
        self.currentPos = wx.StaticText(panel, wx.ID_ANY, label='N/A')

        bmp = wx.ArtProvider.GetBitmap(wx.ART_TIP, wx.ART_OTHER, (16, 16))
        currentHeadingIco = wx.StaticBitmap(panel, wx.ID_ANY, bmp)
        currentHeadingLabel = wx.StaticText(panel, wx.ID_ANY, 'Heading: ')
        self.currentHeading = wx.StaticText(panel, wx.ID_ANY, label='N/A')

        currentLabelSizer = wx.BoxSizer(wx.HORIZONTAL)
        currentStateSizer = wx.BoxSizer(wx.HORIZONTAL)
        currentPosSizer = wx.BoxSizer(wx.HORIZONTAL)
        currentHeadingSizer = wx.BoxSizer(wx.HORIZONTAL)
        
        currentLabelSizer.Add(currentLabel, 0, wx.EXPAND | wx.CENTER, 5)

        currentStateSizer.Add(currentStateIco, 0, wx.ALL, 5)
        currentStateSizer.Add(currentStateLabel, 0, wx.ALL, 5)
        currentStateSizer.Add(self.currentState, 1, wx.ALL|wx.EXPAND, 5)

        currentPosSizer.Add(currentPosIco, 0, wx.ALL, 5)
        currentPosSizer.Add(currentPosLabel, 0, wx.ALL, 5)
        currentPosSizer.Add(self.currentPos, 1, wx.ALL|wx.EXPAND, 5)

        currentHeadingSizer.Add(currentHeadingIco, 0, wx.ALL, 5)
        currentHeadingSizer.Add(currentHeadingLabel, 0, wx.ALL, 5)
        currentHeadingSizer.Add(self.currentHeading, 1, wx.ALL|wx.EXPAND, 5)

        v2a.Add(currentLabelSizer, 0, wx.ALL|wx.EXPAND, 5)

        v2a.Add(currentStateSizer, 0, wx.ALL|wx.EXPAND, 5)
        v2a.Add(currentPosSizer, 0, wx.ALL|wx.EXPAND, 5)
        v2a.Add(currentHeadingSizer, 0, wx.ALL|wx.EXPAND, 5)


        v2b = wx.BoxSizer(wx.VERTICAL)

        nextLabel = wx.StaticText(panel, wx.ID_ANY, 'Next')

        bmp = wx.ArtProvider.GetBitmap(wx.ART_TIP, wx.ART_OTHER, (16, 16))
        nextStateIco = wx.StaticBitmap(panel, wx.ID_ANY, bmp)
        nextStateLabel = wx.StaticText(panel, wx.ID_ANY, 'State: ')
        self.nextState = wx.StaticText(panel, wx.ID_ANY, label='N/A')

        bmp = wx.ArtProvider.GetBitmap(wx.ART_TIP, wx.ART_OTHER, (16, 16))
        nextPosIco = wx.StaticBitmap(panel, wx.ID_ANY, bmp)
        nextPosLabel = wx.StaticText(panel, wx.ID_ANY, 'Position: ')
        self.nextPos = wx.StaticText(panel, wx.ID_ANY, label='N/A')

        bmp = wx.ArtProvider.GetBitmap(wx.ART_TIP, wx.ART_OTHER, (16, 16))
        nextHeadingIco = wx.StaticBitmap(panel, wx.ID_ANY, bmp)
        nextHeadingLabel = wx.StaticText(panel, wx.ID_ANY, 'Heading: ')
        self.nextHeading = wx.StaticText(panel, wx.ID_ANY, label='N/A')

        nextLabelSizer = wx.BoxSizer(wx.HORIZONTAL)

        nextStateSizer = wx.BoxSizer(wx.HORIZONTAL)
        nextPosSizer = wx.BoxSizer(wx.HORIZONTAL)
        nextHeadingSizer = wx.BoxSizer(wx.HORIZONTAL)
        
        nextLabelSizer.Add(nextLabel, 0, wx.EXPAND | wx.CENTER, 5)

        nextStateSizer.Add(nextStateIco, 0, wx.ALL, 5)
        nextStateSizer.Add(nextStateLabel, 0, wx.ALL, 5)
        nextStateSizer.Add(self.nextState, 1, wx.ALL|wx.EXPAND, 5)

        nextPosSizer.Add(nextPosIco, 0, wx.ALL, 5)
        nextPosSizer.Add(nextPosLabel, 0, wx.ALL, 5)
        nextPosSizer.Add(self.nextPos, 1, wx.ALL|wx.EXPAND, 5)

        nextHeadingSizer.Add(nextHeadingIco, 0, wx.ALL, 5)
        nextHeadingSizer.Add(nextHeadingLabel, 0, wx.ALL, 5)
        nextHeadingSizer.Add(self.nextHeading, 1, wx.ALL|wx.EXPAND, 5)
   
        v2b.Add(nextLabelSizer, 0, wx.ALL|wx.EXPAND, 5)

        v2b.Add(nextStateSizer, 0, wx.ALL|wx.EXPAND, 5)
        v2b.Add(nextPosSizer, 0, wx.ALL|wx.EXPAND, 5)
        v2b.Add(nextHeadingSizer, 0, wx.ALL|wx.EXPAND, 5)

        v2ab.Add(v2a, 0,  wx.EXPAND, 10)
        v2ab.Add(v2b, 10,  wx.EXPAND, 2)
        
        v2.Add(v2ab, 0, wx.EXPAND, 5)

        h2.Add(v1, 0, wx.ALL | wx.EXPAND, 10)
        h2.Add(v2, 10, wx.ALL | wx.EXPAND, 20)

        h3 = wx.BoxSizer(wx.HORIZONTAL)
        v3 = wx.BoxSizer(wx.VERTICAL)

        self.textbox = wx.TextCtrl(panel, id=wx.ID_ANY, style=wx.TE_MULTILINE | wx.EXPAND | wx.TE_READONLY, size=wx.Size(12,12))
        self.textbox.SetMinSize((-1, 200))
        self.textbox.SetMaxSize((-1, 200))

        v3.Add(self.textbox, 0, wx.ALL | wx.EXPAND, 1)
        h3.Add(v3, 10, wx.ALL | wx.EXPAND, 20)

        h1.Add(h2, 0, wx.ALL | wx.EXPAND, 1)
        h1.Add(wx.StaticLine(panel), 0, wx.ALL | wx.EXPAND, 1)
        h1.Add(h3, 0, wx.ALL | wx.EXPAND, 10)
      
        panel.SetSizer(h1)
        h1.Fit(panel)
        
        # self.Centre()
        self.Show(True)

        panel.Bind(wx.EVT_BUTTON, self.OnBtnSubmit)
        EVT_RESULT(self, self.onRobotData)

    def onRobotData(self, data: RobotDataEvent):
        # print(data)
        if data.error:
            data.data = "[ERROR] - " + data.data

        self.textbox.AppendText(data.data + '\n')
        # self.textbox.AppendText(f"State: STRAIGHT, {self.shortestPathData[i]}: {self.shortestPathData[i+1]}, Head: {mapping.nodes[self.shortestPathData[i]][self.shortestPathData[i+1]][1]} -> {mapping.nodes[self.shortestPathData[i]][self.shortestPathData[i+1]][1]}, [1, 1, 1, 1, 1], [999, 999, 999, 999, 999]\n")

        if self.autoscroll:
            self.textbox.SetInsertionPointEnd()


        
    def InitPlot(self):
        # TODO: Remove this
        self.autoscroll = True


        self.xdataPoints = [coord[0] for coord in mapping.plottingData.values()]
        self.ydataPoints = [coord[1] for coord in mapping.plottingData.values()]
        self.points = self.ax.scatter(self.xdataPoints, self.ydataPoints, lw=3, color="red")

        self.shortestPathData = None
        self.shortestPath, = self.ax.plot([],[], lw=3, color="blue")
        self.startEndpoints = self.ax.scatter([],[], color="blue", lw=4)
        
        self.robotPos, = self.ax.plot([],[], lw=4, color="green")

        for letter, (x, y) in mapping.plottingData.items():
            self.ax.text(x+1/20, y+1/20, letter, fontsize=12, ha='center', va='center')
    
        self.ax.grid()
        
        # self.ani = animation.FuncAnimation(self.fig, self.UpdatePlot, frames=13, interval=1000)
    
    def SetShortedPath(self, path: list[str]):
        self.shortestPathData = path

        xB, yB = [], []
        for node in path:
            x, y = mapping.plottingData[node]
            xB.append(x)
            yB.append(y)

        self.shortestPath.set_data(xB, yB)
        self.startEndpoints.set_offsets(np.column_stack(([xB[0], xB[-1]], [yB[0], yB[-1]])))

        return self.shortestPath, self.startEndpoints,

    def SetRobotPos(self, pos: tuple[str,str]):
        a, b = mapping.plottingData[pos[0]], mapping.plottingData[pos[1]]
        x,y = [a[0], b[0]],[a[1], b[1]]

        self.robotPos.set_data(x,y)

        return self.robotPos, 

    def UpdatePlot(self, i):

        if i >= len(self.shortestPathData) -1:

            return self.points, 
        
        self.SetRobotPos((self.shortestPathData[i], self.shortestPathData[i+1]))

        self.currentState.SetLabel(self.nextState.GetLabel() if self.nextState.GetLabel() != "" else "STRAIGHT")
        self.currentHeading.SetLabel(mapping.nodes[self.shortestPathData[i]][self.shortestPathData[i+1]][1])
        self.currentPos.SetLabel(f"{self.shortestPathData[i]}-{self.shortestPathData[i+1]}")

        self.nextHeading.SetLabel(mapping.nodes[self.shortestPathData[i+1]][self.shortestPathData[i+2]][1])
        self.nextPos.SetLabel(f"{self.shortestPathData[i+1]}-{self.shortestPathData[i+2]}")
        self.nextState.SetLabel(mapping.HeadingsToState[self.currentHeading.GetLabel()][self.nextHeading.GetLabel()])

        # self.textbox.AppendText(f"State: STRAIGHT, {self.shortestPathData[i]}: {self.shortestPathData[i+1]}, Head: {mapping.nodes[self.shortestPathData[i]][self.shortestPathData[i+1]][1]} -> {mapping.nodes[self.shortestPathData[i]][self.shortestPathData[i+1]][1]}, [1, 1, 1, 1, 1], [999, 999, 999, 999, 999]\n")
        
        # if self.autoscroll: 
        #     self.textbox.SetInsertionPointEnd()

        return self.points, 

if __name__ == '__main__':
    app = wx.App()
    p = RealTimePlot()
    app.MainLoop()
