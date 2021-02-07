#Author-melonTai
#Description-xrotorの設計ファイルと翼型datファイルから3Dモデルを作成するfusionスクリプト

# memo
# 注意：描画するときは、座標を_scalae(=10)で割る
# fusionはプログラム中の数値をcm単位で読み取るため
import adsk.core, adsk.fusion, traceback, math, os

_app = None
_ui  = None
_rowNumber = 0
_scale = 10


class PropDesign():
    def __init__(self):

        # リブのオフセット(外皮の厚み分)[mm]
        self.rib_offset = 1
        # 設計ファイル読み込み(xrotorのrestartfile)
        self.filename = r"restartfile"
        # サブ翼型のdatファイルパス(ペラ根本、ペラ端で使用)
        self.sub_foil_path = r"subfoil.dat"
        # メイン翼型のdatファイルパス(ペラ中央で使用)
        self.main_foil_path = r"mainfoil.dat"
        # ハブ半径[mm]
        self.rib_start = 133
        # リブ間[mm](リブ厚を無視して)
        self.rib_interval = 100
        # 桁位置
        self.rib_center = 0.25
        # 混合比
        self.airfoil_mix_ratio = [100,95,90,85,80,75,70,65,60,55,50,45,40,35,30,25,20,15,10,10,20,30,40,50,60,70,80,90,100,100,100,100,100]
        # 混合するリブ番号
        self.airfoil_mix_number = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,30,31,32,33,34,35,36,37,38,39,40,41,42,43]
        # 翼型の座標点数
        self.dat_amount = 100

        self.XDAT_U=[x/int(self.dat_amount/2) for x in range(1, int(self.dat_amount/2) + 1)]#[0.0002,0.0003,0.0004,0.0005,0.0006,0.0007,0.0008,0.0009,0.001,0.002,0.003,0.004,0.005,0.006,0.007,0.008,0.009,0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.09, 0.1, 0.11, 0.12, 0.13, 0.14, 0.15, 0.16, 0.17, 0.18, 0.19, 0.2, 0.21, 0.22, 0.23, 0.24, 0.25, 0.26, 0.27, 0.28, 0.29, 0.3, 0.31, 0.32, 0.33, 0.34, 0.35, 0.36, 0.37, 0.38, 0.39, 0.4, 0.41, 0.42, 0.43, 0.44, 0.45, 0.46, 0.47, 0.48, 0.49, 0.5, 0.52, 0.54, 0.56, 0.58, 0.6, 0.62, 0.64, 0.66, 0.68, 0.7, 0.72, 0.74, 0.76, 0.78, 0.8, 0.82, 0.84, 0.86, 0.88, 0.9, 0.92, 0.94, 0.96, 0.98,1.0]

        self.XDAT_D=[x/int(self.dat_amount/2) for x in reversed(range(0, int(self.dat_amount/2) + 1))]#[1.0, 0.98, 0.96, 0.94, 0.92, 0.9, 0.88, 0.86, 0.84, 0.82, 0.8, 0.78, 0.76, 0.74, 0.72, 0.7, 0.68, 0.66, 0.64, 0.62, 0.6, 0.58, 0.56, 0.54, 0.52, 0.5, 0.49, 0.48, 0.47, 0.46, 0.45, 0.44, 0.43, 0.42, 0.41, 0.4, 0.39, 0.38, 0.37, 0.36, 0.35, 0.34, 0.33, 0.32, 0.31, 0.3, 0.29, 0.28, 0.27, 0.26, 0.25, 0.24, 0.23, 0.22, 0.21, 0.2, 0.19, 0.18, 0.17, 0.16, 0.15, 0.14, 0.13, 0.12, 0.11, 0.1, 0.09, 0.08, 0.07, 0.06, 0.05, 0.04, 0.03, 0.02, 0.01,0.009,0.008,0.007,0.006,0.005,0.004,0.003,0.002,0.001,0.0009,0.0008,0.0007,0.0006,0.0005,0.0004,0.0003,0.0002, 0.0]

        self.check = {"profile" : True, "rail" : True, "loft": False}

    def linear(self,xs, ys, xn):
        """
            内分する関数(xs昇順)

            Params
            ------
            xs : float list
                翼型のx座標(昇順)
            ys : float list
                翼型のy座標
            xn : float
                内分点

            Returns
            -------
            float
                xnで内分した時のyの値。内分不可の時は、ys[0]。
                また、翼型の前縁または後縁にあたるx座標が、引数に渡されたときは、0
        """
        if(xn == 0.0 or xn == 1.0):
            return 0.0
        for i in range(len(xs)-1):
            if(xs[i] < xn and xn < xs[i+1]):
                return ((xs[i+1] - xn) * ys[i] + (xn - xs[i]) * ys[i+1]) / (xs[i+1] - xs[i])
            if(xs[i] == xn):
                return ys[i]
        print('none data')
        return ys[0]


    def linear_reverse(self,xs, ys, xn):
        """
            内分する関数(xs降順)

            Params
            ------
            xs : float list
                翼型のx座標(降順)
            ys : float list
                翼型のy座標
            xn : float
                内分点

            Returns
            -------
            float
                xnで内分した時のyの値。内分不可の時は、ys[0]。
                また、翼型の前縁または後縁にあたるx座標が、引数に渡されたときは、0
        """
        if(xn == 0.0 or xn == 1.0):
            return 0.0
        for i in range(len(xs)-1):
            if(xs[i] > xn and xn > xs[i+1]):
                #線形補完(内分している)
                return ((xs[i+1] - xn) * ys[i] + (xn - xs[i]) * ys[i+1]) / (xs[i+1] - xs[i])
            if(xs[i] == xn):
                return ys[i]
        print('none data reverse')
        return ys[0]

    def shape_dat(self,datlist):
        """
            翼型の座標位置をXDAT_U、XDAT_Dに揃える関数

            Params
            ------
            datlist : list
                [[x1,y1],[x2,y2],...]

            Returns
            -------
            list
                [[XDAT_D[0],newy[0]],[XDAT_D[1],newy[1]],
                ...,[XDAT_D[-1],newy[m],[XDAT_U[0],newy[m+1]],[XDAT_U[1],newy[m+2]],
                ...,[XDAT_U[-1],newy[-1]]]
        """
        datlist_shaped = []
        datlist_x = [dat[0] for dat in datlist]
        datlist_y = [dat[1] for dat in datlist]
        for x in self.XDAT_D:

            datlist_shaped.append([x,self.linear_reverse(datlist_x, datlist_y,x)])
        for x in self.XDAT_U:
            datlist_shaped.append([x,self.linear(datlist_x, datlist_y,x)])
        return datlist_shaped

    def getCenterThickness(self,airfoil, c):#中心線のy座標を求める
        """
        airfoilに与えられた翼型の中心線についてcに与えられたx座標に対応するy座標を得る

        Params
        ------
        airfoil : float list
            翼型の座標
            [[x1,y1],[x2,y2],...]
        c : float
            知りたいx座標(0 < c < 1)
        Returns
        -------
        float
            cに対応するy座標
        """
        p = []
        for i in range(len(airfoil) - 2):
            if (airfoil[i][0] < c and c < airfoil[i+1][0]) or (airfoil[i+1][0] < c and c < airfoil[i][0]):
                m = (c - airfoil[i][0]) / (airfoil[i+1][0] - airfoil[i][0])
                p.append(airfoil[i][1] * (1 - m) + airfoil[i+1][1] * m)
            elif (airfoil[i][0] == c):
                p.append(airfoil[i][1])
        if len(p) == 2:
            return (p[0] + p[1]) / 2.0
        else:
            return 0

    def interpolate_dat(self,datlist_shaped_list, propotions):
        """
            翼型を混合する関数
            Params
            ------
            datlist_list : float list
                混合する翼型の座標リスト
                shape_datを通すこと
                [
                    [   # 翼型1の座標
                        [x11, y11],
                        [x12, y12],
                        ...
                    ],
                    [   # 翼型2の座標
                        [x21, y21],
                        [x22, y22],
                        ...
                    ],
                    ...
                ]

            propotions : float list
                各翼型の混合比率(百分率)
                例:
                    翼型1:翼型2:翼型3 = 0.2 : 0.3 : 0.5
                    で混合するとき引数は
                    [0.2, 0.3, 0.5]
                混合比率の合計は1になるよう注意

            Returns
            -------
                float list
                混合した翼型の座標
                [[newx1,newy1],[newx2,newy2],...]
        """
        datlist_new_y = [0]*len(datlist_shaped_list[0])
        datlist_new_x = [dat[0] for dat in datlist_shaped_list[0]]
        for datlist_shaped, p in zip(datlist_shaped_list, propotions):
            datlist_new_y = [dat[1]*p + dat_new_y for dat, dat_new_y in zip(datlist_shaped,datlist_new_y)]
        datlist_new = [[dat_new_x,dat_new_y] for dat_new_x, dat_new_y in zip(datlist_new_x, datlist_new_y)]

        return datlist_new

    def set_scale(self,ps,scale):
        return [[p[0] / scale, p[1] / scale] for p in ps]

    def build(self):
        global _app, _ui, _scale

        _app = adsk.core.Application.get()
        _ui = _app.userInterface

        doc = _app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = _app.activeProduct

        # Get the root component of the active design.
        rootComp = design.rootComponent

        # Set styles of progress dialog.
        progressDialog = _ui.createProgressDialog()
        progressDialog.cancelButtonText = 'Cancel'
        progressDialog.isBackgroundTranslucent = False
        progressDialog.isCancelButtonShown = True

        f=open(self.filename)
        fd = f.read()
        f.close()
        lines = fd.split('\n')
        blade_radius = 1000 * float(lines[5].split()[0])# 半径を設計ファイルから取得
        #print("blade_radius",blade_radius)
        design_data_r = []
        design_data_c = []
        design_data_rot = []
        skip = 9 + int(lines[9].split()[0])*10 + 6
        for line in lines[skip:]:
            d = line.split()
            if(len(d) == 4):
                design_data_r.append(float(d[0]) * blade_radius)
                design_data_c.append(float(d[1]) * blade_radius)
                design_data_rot.append(float(d[2])*math.pi/180)

        f=open(self.sub_foil_path)
        ad = f.read()
        f.close()
        lines = ad.split('\n')
        _sub_foil = []
        for line in lines[1:]:
            d = line.split()
            if(len(d) == 2):
                _sub_foil.append([float(d[0]), float(d[1])])
        sub_foil = self.shape_dat(_sub_foil)

        f=open(self.main_foil_path)
        ad = f.read()
        f.close()
        lines = ad.split('\n')
        _main_foil = []
        for line in lines[1:]:
            d = line.split()
            if(len(d) == 2):
                _main_foil.append([float(d[0]), float(d[1])])
        main_foil = self.shape_dat(_main_foil)

        output_rib_data = ""
        x = self.rib_start
        rot_offset = (-design_data_rot[0] - design_data_rot[-1] + math.pi) / 2
        rib_number = 0
        last_rearR = -1
        last_rearL = -1

        # Show dialog
        progressDialog.show('profile', 'Percentage: %p, Current Value: %v, Total steps: %m', 0, int((blade_radius-self.rib_start)/self.rib_interval), 1)

        line_rib_collections = []
        sketch_rib_collections = []
        dat_list = []

        while x < blade_radius:
            # If progress dialog is cancelled, stop drawing.
            if progressDialog.wasCancelled:
                break
            # chord
            cmod = self.linear(design_data_r, design_data_c, x)
            # pitch
            rot = -self.linear(design_data_r, design_data_rot, x) - rot_offset

            # interpolate airfoil
            airfoil_data = []
            if rib_number in self.airfoil_mix_number:
                mix = self.airfoil_mix_ratio[self.airfoil_mix_number.index(rib_number)] / 100
            else:
                mix = 0
            #print(mix)
            #mix = 0
            airfoil_data = self.interpolate_dat([sub_foil,main_foil],[mix,1-mix])
            rib_center_camber = self.getCenterThickness(airfoil_data, self.rib_center)

            # rotate and expand airfoil
            airfoil_poly = []
            rear_airfoil_poly = []
            #print(self.rib_center)

            for p in airfoil_data:
                px = (p[0] - self.rib_center) * cmod
                py = (p[1] - rib_center_camber) * cmod
                nx = px * math.cos(rot) - py * math.sin(rot) + x
                ny = px * math.sin(rot) + py * math.cos(rot)
                airfoil_poly.append([nx, ny])

            # scale
            rib_poly_cm = self.set_scale(airfoil_poly, _scale)

            dat_list.append([dat + [x/_scale] for dat in rib_poly_cm])



            if self.check["profile"]:

                # draw profile
                sketch_rib_collections.append(rootComp.sketches.add(rootComp.xYConstructionPlane))
                sketch_rib = sketch_rib_collections[-1]
                sketch_rib.name = 'rib{}'.format(rib_number)
                line_rib_collections.append(sketch_rib.sketchCurves.sketchLines)
                line_rib_collection = line_rib_collections[-1]

                line = line_rib_collection.addByTwoPoints(\
                    adsk.core.Point3D.create(rib_poly_cm[0][0], rib_poly_cm[0][1], x/_scale), \
                    adsk.core.Point3D.create(rib_poly_cm[1][0], rib_poly_cm[1][1], x/_scale)\
                )
                for p in rib_poly_cm[2:]:
                    line = line_rib_collection.addByTwoPoints(\
                        line.endSketchPoint,\
                        adsk.core.Point3D.create(p[0], p[1], x/_scale)\
                    )
                line = line_rib_collection.addByTwoPoints(\
                    line.endSketchPoint,\
                    adsk.core.Point3D.create(rib_poly_cm[0][0], rib_poly_cm[0][1], x/_scale)\
                )

            x += self.rib_interval
            rib_number += 1
            # Update progress value of progress dialog
            progressDialog.progressValue = rib_number + 1

        progressDialog.hide()
        if self.check["rail"]:
            # レール生成
            splines = []
            for i in range(len(dat_list[0])-1):
                splines.append([a[i] for a in dat_list])
            # Show dialog
            progressDialog.show('rail', 'Percentage: %p, Current Value: %v, Total steps: %m', 0, len(splines), 1)
            count = 0
            sketch_curve = rootComp.sketches.add(rootComp.xYConstructionPlane)
            # Create a new sketch on the xy plane.
            for spline in splines:
                # If progress dialog is cancelled, stop drawing.
                if progressDialog.wasCancelled:
                    break
                # Create an object collection for the points.
                points = adsk.core.ObjectCollection.create()
                for p in spline:
                    # Define the points the spline with fit through.
                    points.add(adsk.core.Point3D.create(p[0], p[1], p[2]))
                # Create the spline.
                sketch_curve.sketchCurves.sketchFittedSplines.add(points)
                count += 1
                progressDialog.progressValue = count
            # Hide the progress dialog at the end.
            progressDialog.hide()

        if self.check["loft"]:
            # ロフト生成
            # Create loft feature input
            loftFeats = rootComp.features.loftFeatures
            loftInput = loftFeats.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
            loftSectionsObj = loftInput.loftSections
            # Show dialog
            progressDialog.show('loft', 'Percentage: %p, Current Value: %v, Total steps: %m', 0, len(sketch_rib_collections), 1)
            count = 0
            for sketch in sketch_rib_collections:
                # If progress dialog is cancelled, stop drawing.
                if progressDialog.wasCancelled:
                    break
                loftSectionsObj.add(sketch.profiles.item(0))
                count += 1
                progressDialog.progressValue = count
            loftInput.isSolid = False
            # Create loft feature
            loftFeats.add(loftInput)
            # Hide the progress dialog at the end.
            progressDialog.hide()


# Global set of event handlers to keep them referenced for the duration of the command
_handlers = []
_prop = PropDesign()

def fileOpen(input,ftype = '*'):
    # Set styles of file dialog.
    fileDlg = _ui.createFileDialog()
    fileDlg.isMultiSelectEnabled = False
    fileDlg.title = 'Fusion File Dialog'
    fileDlg.filter = '*.{}'.format(ftype)

    # Show file open dialog
    dlgResult = fileDlg.showOpen()
    if dlgResult == adsk.core.DialogResults.DialogOK:
        #_ui.messageBox(fileDlg.filenames[0])
        input.value = fileDlg.filenames[0]
    else:
        input.value = "error"

# Adds a new row to the table.
def addRowToTable(tableInput):
    global _rowNumber
    # Get the CommandInputs object associated with the parent command.
    cmdInputs = adsk.core.CommandInputs.cast(tableInput.commandInputs)

    # Create three new command inputs.
    stringInput1 =  cmdInputs.addStringValueInput('TableInput1_string{}'.format(_rowNumber), 'rib_number', str(_rowNumber))
    stringInput2 =  cmdInputs.addStringValueInput('TableInput2_string{}'.format(_rowNumber), 'mix_ratio', str(0))

    # Add the inputs to the table.
    row = tableInput.rowCount
    tableInput.addCommandInput(stringInput1, row, 0)
    tableInput.addCommandInput(stringInput2, row, 1)

    # Increment a counter used to make each row unique.
    _rowNumber = _rowNumber + 1

def is_int(s):
    try:
        int(s)
        return True
    except ValueError:
        return False
def is_float(s):
    try:
        float(s)
        return True
    except ValueError:
        return False

# Event handler that reacts to any changes the user makes to any of the command inputs.
class MyCommandInputChangedHandler(adsk.core.InputChangedEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self, args):
        try:
            global _prop, _scale
            eventArgs = adsk.core.InputChangedEventArgs.cast(args)
            inputs = eventArgs.inputs
            cmdInput = eventArgs.input
            #_ui.messageBox(str(cmdInput.parentCommandInput.id))
            if cmdInput.parentCommandInput != None:
                if cmdInput.parentCommandInput.id == 'tab_1':
                    if cmdInput.id == 'import_xrot':
                        fileOpen(inputs.itemById('xrotor_restartfile'))
                    elif cmdInput.id == 'import_main':
                        fileOpen(inputs.itemById('main_foil_path'), 'dat')
                    elif cmdInput.id == 'import_sub':
                        fileOpen(inputs.itemById('sub_foil_path'), 'dat')
                    for input in inputs:
                        if input.id == 'xrotor_restartfile':
                            _prop.filename = input.value
                        elif input.id == 'main_foil_path':
                            _prop.main_foil_path = input.value
                        elif input.id == 'sub_foil_path':
                            _prop.sub_foil_path = input.value
                        elif input.id == 'hub_radius':
                            _prop.rib_start = input.value * _scale
                        elif input.id == 'profile_interval':
                            _prop.rib_interval = input.value * _scale
                        elif input.id == 'center' and is_float(input.value):
                            _prop.rib_center = float(input.value)
                        elif input.id == 'dat_amount' and is_int(input.value):
                            _prop.dat_amount = int(input.value)
                            _prop.XDAT_U=[x/int(_prop.dat_amount/2) for x in range(1, int(_prop.dat_amount/2) + 1)]
                            _prop.XDAT_D=[x/int(_prop.dat_amount/2) for x in reversed(range(0, int(_prop.dat_amount/2) + 1))]

                elif cmdInput.parentCommandInput.id == 'table':
                    tableInput = inputs.itemById('table')
                    if cmdInput.id == 'tableAdd':
                        addRowToTable(tableInput)
                    elif cmdInput.id == 'tableDelete':
                        if tableInput.selectedRow == -1:
                            _ui.messageBox('Select one row to delete.')
                        elif tableInput.selectedRow == 0:
                            _ui.messageBox('cannot delete this row')
                        else:
                            tableInput.deleteRow(tableInput.selectedRow)
                    _prop.airfoil_mix_number = []
                    _prop.airfoil_mix_ratio = []
                    row = tableInput.rowCount
                    for r in range(1,row):
                        if is_int(tableInput.getInputAtPosition(r, 0).value):
                            _prop.airfoil_mix_number.append(int(tableInput.getInputAtPosition(r, 0).value))
                        if is_float(tableInput.getInputAtPosition(r, 1).value):
                            _prop.airfoil_mix_ratio.append(float(tableInput.getInputAtPosition(r, 1).value))

                elif cmdInput.parentCommandInput.id == 'tab_4':
                    for input in inputs:
                        if input.id == 'profile':
                            _prop.check["profile"] = input.value
                        elif input.id == 'rail':
                            _prop.check["rail"] = input.value
                        elif input.id == 'loft':
                            _prop.check["loft"] = input.value

                elif cmdInput.parentCommandInput.id == 'tab_3':
                    if cmdInput.id == 'build':
                        _prop.build()
                        #_ui.messageBox(str(_prop.check))




        except:
            _ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


# Event handler that reacts to when the command is destroyed. This terminates the script.
class MyCommandDestroyHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self, args):
        try:
            # When the command is done, terminate the script
            # This will release all globals which will remove all event handlers
            adsk.terminate()
        except:
            _ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


# Event handler that reacts when the command definitio is executed which
# results in the command being created and this event being fired.
class MyCommandCreatedHandler(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self, args):
        try:
            # Get the command that was created.
            cmd = adsk.core.Command.cast(args.command)

            # Connect to the command destroyed event.
            onDestroy = MyCommandDestroyHandler()
            cmd.destroy.add(onDestroy)
            _handlers.append(onDestroy)

            # Connect to the input changed event.
            onInputChanged = MyCommandInputChangedHandler()
            cmd.inputChanged.add(onInputChanged)
            _handlers.append(onInputChanged)

            # Get the CommandInputs collection associated with the command.
            inputs = cmd.commandInputs

            # Create a tab input.
            tabCmdInput1 = inputs.addTabCommandInput('tab_1', 'param')
            tab1ChildInputs = tabCmdInput1.children

            # Create a string value input.
            xrot_file_input = tab1ChildInputs.addStringValueInput('xrotor_restartfile', 'xrotor_restartfile', _prop.filename)

            # Create bool value input with button style that can be clicked.
            tab1ChildInputs.addBoolValueInput('import_xrot', 'import_xrot', False, '', True)

            # Create a string value input.
            main_foil_input = tab1ChildInputs.addStringValueInput('main_foil_path', 'main_foil_path', _prop.main_foil_path)

            # Create bool value input with button style that can be clicked.
            tab1ChildInputs.addBoolValueInput('import_main', 'import_main', False, '', True)

            # Create a string value input.
            sub_foil_input = tab1ChildInputs.addStringValueInput('sub_foil_path', 'sub_foil_path', _prop.sub_foil_path)

            # Create bool value input with button style that can be clicked.
            tab1ChildInputs.addBoolValueInput('import_sub', 'import_sub', False, '', True)

            # Create value input.
            rib_start_input = tab1ChildInputs.addValueInput('hub_radius', 'hub_radius', 'mm', adsk.core.ValueInput.createByReal(_prop.rib_start / _scale))

            # Create value input.
            rib_interval_input = tab1ChildInputs.addValueInput('profile_interval', 'profile_interval', 'mm', adsk.core.ValueInput.createByReal(_prop.rib_interval / _scale))

            # Create value input.
            rib_center_input = tab1ChildInputs.addStringValueInput('center', 'center', str(_prop.rib_center))

            # Create value input.
            rib_center_input = tab1ChildInputs.addStringValueInput('dat_amount', 'dat_amount', str(_prop.dat_amount))

            # Create tab input 2
            tabCmdInput2 = inputs.addTabCommandInput('tab_2', 'mix foil')
            tab2ChildInputs = tabCmdInput2.children

            # Create table input
            tableInput = tab2ChildInputs.addTableCommandInput('table', 'Table', 2, '1:1')
            tableInput.maximumVisibleRows = 20
            global _rowNumber
            cmdInputs = adsk.core.CommandInputs.cast(tableInput.commandInputs)
            head1 = cmdInputs.addTextBoxCommandInput('profile_number', 'profile number', 'profile number', 1, True)
            head2 = cmdInputs.addTextBoxCommandInput('mix_ratio', 'subfoil mix ratio', 'sub mix ratio[%]', 1, True)
            tableInput.addCommandInput(head1, 0, 0)
            tableInput.addCommandInput(head2, 0, 1)
            _rowNumber = _rowNumber + 1
            for n, r in zip(_prop.airfoil_mix_number, _prop.airfoil_mix_ratio):
                cmdInputs = adsk.core.CommandInputs.cast(tableInput.commandInputs)
                stringInput1 =  cmdInputs.addStringValueInput('TableInput1_string{}'.format(_rowNumber), 'rib_number', str(n))
                stringInput2 =  cmdInputs.addStringValueInput('TableInput2_string{}'.format(_rowNumber), 'mix_ratio', str(r))
                row = tableInput.rowCount
                tableInput.addCommandInput(stringInput1, row, 0)
                tableInput.addCommandInput(stringInput2, row, 1)
                _rowNumber = _rowNumber + 1

            # Add inputs into the table.
            addButtonInput = tab2ChildInputs.addBoolValueInput('tableAdd', 'Add', False, '', True)
            tableInput.addToolbarCommandInput(addButtonInput)
            deleteButtonInput = tab2ChildInputs.addBoolValueInput('tableDelete', 'Delete', False, '', True)
            tableInput.addToolbarCommandInput(deleteButtonInput)

            # Create tab input 4
            tabCmdInput4 = inputs.addTabCommandInput('tab_4', 'option')
            tab4ChildInputs = tabCmdInput4.children
            tab4ChildInputs.addBoolValueInput('profile', 'profile', True, '', _prop.check["profile"])
            tab4ChildInputs.addBoolValueInput('rail', 'rail', True, '', _prop.check["rail"])
            tab4ChildInputs.addBoolValueInput('loft', 'loft', True, '', _prop.check["loft"])

            # Create tab input 3
            tabCmdInput3 = inputs.addTabCommandInput('tab_3', 'build')
            tab3ChildInputs = tabCmdInput3.children
            # Create bool value input with button style that can be clicked.
            tab3ChildInputs.addBoolValueInput('build', 'build', False, '', True)

        except:
            _ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

def run(context):
    try:
        global _app, _ui
        _app = adsk.core.Application.get()
        _ui = _app.userInterface

        # Get the existing command definition or create it if it doesn't already exist.
        cmdDef = _ui.commandDefinitions.itemById('xrotToFusionSketch')
        if not cmdDef:
            cmdDef = _ui.commandDefinitions.addButtonDefinition('XrotorTo3DModel', 'Xrotor To 3DModel', 'Sample to demonstrate various command inputs.')

        # Connect to the command created event.
        onCommandCreated = MyCommandCreatedHandler()
        cmdDef.commandCreated.add(onCommandCreated)
        _handlers.append(onCommandCreated)

        # Execute the command definition.
        cmdDef.execute()

        # Prevent this module from being terminated when the script returns, because we are waiting for event handlers to fire.
        adsk.autoTerminate(False)
    except:
        if _ui:
            _ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
