# import struct
#
# data = [
#     999,999,999,999,999, #5i
#     0,0,0,0,0, #5?
#
#     "RIGHT_CORNER".encode(), #C State 21
#     "STRAIGHT".encode(), #N State
#     "E".encode(), #C Heading
#     "S".encode(), #N Headign
#     "E-S".encode(), #C Pos
#     "S-Z".encode(), #N Pos
# ]
# # input_string[:21].ljust(21, b'\0'.decode())
#
# format_string = "<5i5?12s12s1s1s3s3s"
# # Pack the data into bytes
# packed_data = struct.pack(format_string, *data)
#
# print("Packed bytes:", packed_data)
#
# print(len(data), len(packed_data))
#
# print(struct.unpack(format_string, packed_data))


import wx

class RobotControlPanel(wx.Panel):
    def __init__(self, frame, parent):
        super().__init__(parent)

        self.frame = frame
        self.speed_slider = wx.Slider(self, value=50, minValue=0, maxValue=100, style=wx.SL_HORIZONTAL)
        self.position_label = wx.StaticText(self, label="Position (A..Z-A..Z):")
        self.position_input = wx.TextCtrl(self)
        self.endpoint_label = wx.StaticText(self, label="Endpoint (A..Z):")
        self.endpoint_input = wx.TextCtrl(self)
        self.box_label = wx.StaticText(self, label="Box Placement (A..Z):")
        self.box_input = wx.TextCtrl(self)
        self.direction_label = wx.StaticText(self, label="Direction:")
        self.direction_choices = ['Forward', 'Backward', 'Left', 'Right', 'Stop']
        self.direction_dropdown = wx.Choice(self, choices=self.direction_choices)
        self.move_button = wx.Button(self, label="Move")

        self.__set_properties()
        self.__do_layout()
        self.__bind_events()

    def __set_properties(self):
        self.move_button.SetForegroundColour(wx.Colour(255, 255, 255))
        self.move_button.SetBackgroundColour(wx.Colour(0, 128, 0))

    def __do_layout(self):
        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer.Add(wx.StaticText(self, label="Robot Control Panel"), 0, wx.ALL, 10)
        sizer.Add(self.speed_slider, 0, wx.EXPAND | wx.ALL, 10)
        sizer.Add(self.position_label, 0, wx.LEFT | wx.BOTTOM, 10)
        sizer.Add(self.position_input, 0, wx.EXPAND | wx.ALL, 10)
        sizer.Add(self.endpoint_label, 0, wx.LEFT | wx.BOTTOM, 10)
        sizer.Add(self.endpoint_input, 0, wx.EXPAND | wx.ALL, 10)
        sizer.Add(self.box_label, 0, wx.LEFT | wx.BOTTOM, 10)
        sizer.Add(self.box_input, 0, wx.EXPAND | wx.ALL, 10)
        sizer.Add(self.direction_label, 0, wx.LEFT | wx.BOTTOM, 10)
        sizer.Add(self.direction_dropdown, 0, wx.EXPAND | wx.ALL, 10)
        sizer.Add(self.move_button, 0, wx.EXPAND | wx.ALL, 10)
        self.SetSizer(sizer)
        self.Layout()

    def get_slider_value(self):
        return self.speed_slider.GetValue()

    def get_position_input(self):
        return self.position_input.GetValue()

    def get_endpoint_input(self):
        return self.endpoint_input.GetValue()

    def get_box_input(self):
        return self.box_input.GetValue()

    def __bind_events(self):
        self.move_button.Bind(wx.EVT_BUTTON, self.on_move_button_clicked)

    def on_move_button_clicked(self, event):
        print("te")
        wx.PostEvent(self.frame, DataEvent())

class RobotControlDialog(wx.Dialog):
    def __init__(self, frame ):
        super().__init__(None, title="Robot Control Dialog", size=(1000, 1000))

        self.control_panel = RobotControlPanel(frame, self)

        self.__do_layout()

    def __do_layout(self):
        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer.Add(self.control_panel, 1, wx.EXPAND)
        self.SetSizer(sizer)
        self.Layout()

class DataEvent(wx.PyEvent):
    def __init__(self):
        super().__init__()
        self.SetEventType(wx.NewEventType())
        self.data = "test"

class MainFrame(wx.Frame):
    def __init__(self):
        super().__init__(None, title="Robot Control App", size=(1000, 1000))

        self.control_dialog_button = wx.Button(self, label="Open Control Dialog")
        self.Bind(wx.EVT_BUTTON, self.on_open_dialog, self.control_dialog_button)

        self.result_text = wx.TextCtrl(self, style=wx.TE_MULTILINE|wx.TE_READONLY)

        self.__do_layout()

    def __do_layout(self):
        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer.Add(self.control_dialog_button, 0, wx.ALIGN_CENTER | wx.ALL, 20)
        sizer.Add(self.result_text, 1, wx.EXPAND | wx.ALL, 20)
        self.SetSizer(sizer)
        self.Layout()

    def on_open_dialog(self, event):
        dialog = RobotControlDialog(self)
        dialog.ShowModal()
        dialog.Destroy()

    def on_data_received(self, event):
        self.result_text.AppendText(f"Speed: {event.data['speed']}\n")
        self.result_text.AppendText(f"Position: {event.data['position']}\n")
        self.result_text.AppendText(f"Endpoint: {event.data['endpoint']}\n")
        self.result_text.AppendText(f"Box Placement: {event.data['box_placement']}\n")
        self.result_text.AppendText(f"Direction: {event.data['direction']}\n")
        self.result_text.AppendText('\n')

if __name__ == "__main__":
    app = wx.App(False)
    frame = MainFrame()
    frame.Show(True)
    app.MainLoop()
