import wx
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg as FigureCanvas
import matplotlib.animation as animation

import mapping

class RealTimePlot(wx.Frame):
    def __init__(self):
        super(RealTimePlot, self).__init__(None, title="Plotter", size=(1100, 800))
        
        self.InitUI()
        self.InitPlot() 
        
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
        v2 = wx.BoxSizer(wx.HORIZONTAL)
        
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
   
        nextStateSizer =  wx.BoxSizer(wx.HORIZONTAL)
        nextPosSizer = wx.BoxSizer(wx.HORIZONTAL)
        nextHeadingSizer = wx.BoxSizer(wx.HORIZONTAL)

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

        v2.Add(v2a, 0,  wx.EXPAND, 1)
        v2.Add(v2b, 1,  wx.EXPAND, 2)
        
        
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
        
        self.ani = animation.FuncAnimation(self.fig, self.UpdatePlot, frames=100, interval=1000)
    
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
        self.textbox.AppendText(f"State: STRAIGHT, {self.shortestPathData[i]}: {self.shortestPathData[i+1]}, Head: {mapping.nodes[self.shortestPathData[i]][self.shortestPathData[i+1]][1]} -> {mapping.nodes[self.shortestPathData[i]][self.shortestPathData[i+1]][1]}, [1, 1, 1, 1, 1], [999, 999, 999, 999, 999]\n")
        
        if self.autoscroll: 
            self.textbox.SetInsertionPointEnd()

        return self.points, 

if __name__ == '__main__':
    app = wx.App()

    graph = mapping.Graph(mapping.nodes)

    # graph.BlockConnection("K", "N")
    # graph.BlockConnection("N", "Q")
    # graph.BlockConnection("J", "L")
    # graph.BlockConnection("O", "W")
    # graph.BlockConnection("E", "M")
    # graph.BlockConnection("R", "S")

    # Example usage: find the shortest distance between nodes "A" and "Z"
    start_node = "A"
    end_node = "AA"
    shortest_distance, shortest_path, dirs = graph.dijkstra(start_node, end_node)

    p = RealTimePlot()
    p.SetShortedPath(shortest_path)
    app.MainLoop()
