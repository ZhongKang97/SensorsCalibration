"""
Demonstrate use of GLLinePlotItem to draw cross-sections of a surface.
"""

import numpy as np
import os
from PySide6 import QtWidgets, QtCore, QtGui
# from pyqtgraph.Qt import QtCore,QtGui
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.opengl.GLGraphicsItem import GLGraphicsItem
import OpenGL.GL as ogl
from numpy import ptp
from GeometryLib import euler2Rnb, rotationMatrixToEulerAngles
import transformations as tf
import draw_trajcory_resource
np.set_printoptions(suppress=True)

def strip_first_col(fname, delimiter=None):
    with open(fname, 'r') as fin:
        for line in fin:
            try:
                yield line.split(delimiter, 1)[1]
            except IndexError:
                continue


class drawCoordinateFrameItem(GLGraphicsItem):
    """
    **Bases:** :class:`GLGraphicsItem <pyqtgraph.opengl.GLGraphicsItem>`

    Displays three lines indicating origin and orientation of local coordinate system.

    """

    def __init__(self, rpy, t, id=0, parent: gl.GLViewWidget = None, is_keyframe=False, size=None, axis_width=5,
                 antialias=True, glOptions='translucent'):
        self.is_keyframe = is_keyframe
        self.id = id
        self.parent = parent
        GLGraphicsItem.__init__(self)
        self.axis_width = axis_width
        self.R = euler2Rnb(rpy)
        self.t = t
        if size is None:
            size = QtGui.QVector3D(1, 1, 1)
        self.antialias = antialias
        self.setSize(size=size)
        self.setGLOptions(glOptions)
        self.color = QtCore.Qt.GlobalColor.red
        self.text = ''
        self.font = QtGui.QFont('Helvetica', 16)

    def setSize(self, x=None, y=None, z=None, size=None):
        """
        Set the size of the axes (in its local coordinate system; this does not affect the transform)
        Arguments can be x,y,z or size=QVector3D().
        """
        if size is not None:
            x = size.x()
            y = size.y()
            z = size.z()
        self.__size = [x, y, z]
        self.update()

    def size(self):
        return self.__size[:]

    def paint(self):

        # glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        # glEnable( GL_BLEND )
        # glEnable( GL_ALPHA_TEST )
        self.setupGLState()

        if self.antialias:
            ogl.glEnable(ogl.GL_LINE_SMOOTH)
            ogl.glHint(ogl.GL_LINE_SMOOTH_HINT, ogl.GL_NICEST)

        x, y, z = self.size()
        o = np.array([0, 0, 0])
        x0 = np.array([x, 0, 0])
        y0 = np.array([0, y, 0])
        z0 = np.array([0, 0, z])

        o1 = self.R.dot(o) + self.t
        x1 = self.R.dot(x0) + self.t
        y1 = self.R.dot(y0) + self.t
        z1 = self.R.dot(z0) + self.t

        ogl.glLineWidth(self.axis_width)
        ogl.glBegin(ogl.GL_LINES)

        ogl.glColor4f(0, 1, 0, .6)  # z is green
        ogl.glVertex3f(o1[0], o1[1], o1[2])
        ogl.glVertex3f(z1[0], z1[1], z1[2])

        ogl.glColor4f(1, 1, 0, .6)  # y is yellow
        ogl.glVertex3f(o1[0], o1[1], o1[2])
        ogl.glVertex3f(y1[0], y1[1], y1[2])

        ogl.glColor4f(0, 0, 1, .6)  # x is blue
        ogl.glVertex3f(o1[0], o1[1], o1[2])
        ogl.glVertex3f(x1[0], x1[1], x1[2])
        ogl.glEnd()
        if self.is_keyframe:
            modelview = ogl.glGetDoublev(ogl.GL_MODELVIEW_MATRIX)
            projection = ogl.glGetDoublev(ogl.GL_PROJECTION_MATRIX)

            viewport = [0, 0, self.view().width(), self.view().height()]
            text_pos = self.__project(np.array([o1[0], o1[1], o1[2]]), modelview, projection, viewport)

            text_pos.setY(viewport[3] - text_pos.y())
            painter = QtGui.QPainter(self.view())
            painter.setPen(self.color)
            painter.setFont(self.font)
            painter.setRenderHints(QtGui.QPainter.RenderHint.Antialiasing | QtGui.QPainter.RenderHint.TextAntialiasing)
            painter.drawText(text_pos, 'frame_id:' + str(self.id))
            painter.end()

    def __project(self, obj_pos, modelview, projection, viewport):
        obj_vec = np.append(np.array(obj_pos), [1.0])

        view_vec = np.matmul(modelview.T, obj_vec)
        proj_vec = np.matmul(projection.T, view_vec)

        if proj_vec[3] == 0.0:
            return QtCore.QPointF(0, 0)

        proj_vec[0:3] /= proj_vec[3]

        return QtCore.QPointF(
            viewport[0] + (1.0 + proj_vec[0]) * viewport[2] / 2,
            viewport[1] + (1.0 + proj_vec[1]) * viewport[3] / 2
        )


class TrajectoryViewer(QtWidgets.QWidget):
    def __init__(self):
        QtWidgets.QWidget.__init__(self)
        self.previewLayout = QtWidgets.QGridLayout()
        self.previewLayout.setSpacing(0)
        self.mainLayout = QtWidgets.QVBoxLayout()
        self.initDispW = 2
        self.initDispH = 1
        self.nowDispW = self.initDispW
        self.nowDispH = self.initDispH
        self.vectorDisp = []
        self.vectorDispName = []
        self.vectorDispWiget = []
        self.setUpInterface()
        self.filename = ''

    def setUpInterface(self):
        self.InputFileButton = QtWidgets.QPushButton("打开文件")
        self.InputFileButton.clicked.connect(self.openFile)
        self.DrawButton = QtWidgets.QPushButton("绘制轨迹")
        self.DrawButton.clicked.connect(self.drawData)
        self.formatCombo = QtWidgets.QComboBox()
        self.formatCombo.addItem("[timestamp_dateTimefomat R00 R01 R02 tx R10 R11 R12 ty R20 R21 R22 tz]")
        self.formatCombo.addItem("[timestamp_nanoseconds R00 R01 R02 tx R10 R11 R12 ty R20 R21 R22 tz]")
        self.formatCombo.addItem("[timestamp qx qy qz qw tx ty tz]")
        toplayout = QtWidgets.QHBoxLayout()
        toplayout.addWidget(self.InputFileButton)
        toplayout.addWidget(self.DrawButton)
        self.mainLayout.addLayout(toplayout)
        self.filenameEdit = QtWidgets.QLineEdit()
        self.mainLayout.addWidget(self.filenameEdit)
        self.mainLayout.addWidget(self.formatCombo)
        self.inserIndex_Row = QtWidgets.QLineEdit(str(0))
        self.inserIndex_Col = QtWidgets.QLineEdit(str(0))
        self.skipData = QtWidgets.QLineEdit(str(10))
        self.skipFrame = QtWidgets.QLineEdit(str(10))
        self.keyFrameCount = QtWidgets.QLineEdit(str(50))
        self.insertFrom = QtWidgets.QFormLayout()
        self.insertFrom.addRow("InsertIndexRow", self.inserIndex_Row)
        self.insertFrom.addRow("InsertIndexCol", self.inserIndex_Col)
        self.insertFrom.addRow("skipData", self.skipData)
        self.insertFrom.addRow("skipFrame", self.skipFrame)
        self.insertFrom.addRow("keyFrameCount", self.keyFrameCount)
        self.cfgDispWidthEdit = QtWidgets.QLineEdit(str(self.initDispW))
        self.cfgDispHeightEdit = QtWidgets.QLineEdit(str(self.initDispH))
        self.cfgDispWidthEdit.textChanged.connect(self.BuildDsip)
        self.cfgDispHeightEdit.textChanged.connect(self.BuildDsip)
        self.configFrom = QtWidgets.QFormLayout()
        self.configFrom.addRow("Num Row", self.cfgDispHeightEdit)
        self.configFrom.addRow("Num Col", self.cfgDispWidthEdit)
        self.mainLayout.addLayout(self.insertFrom)
        self.mainLayout.addLayout(self.configFrom)
        self.initBuildDisp()
        self.mainLayout.addLayout(self.previewLayout)
        self.mainLayout.setStretchFactor(self.previewLayout,2)
        self.setLayout(self.mainLayout)

    def initBuildDisp(self):
        for index in range(self.initDispW*self.initDispH):
            w = gl.GLViewWidget()
            w.setCameraPosition(distance=40)
            w.setBackgroundColor(pg.mkColor(150, 150, 150, 0.6))
            self.vectorDisp.append(w)
            nameLable = QtWidgets.QLabel("default")
            nameLable.setAlignment(QtCore.Qt.AlignCenter)
            self.vectorDispName.append(nameLable)
            col = index % self.nowDispW
            row = int(index / self.nowDispW)
            dispWidget = QtWidgets.QWidget()
            layout = QtWidgets.QVBoxLayout()
            layout.addWidget(self.vectorDisp[index])
            layout.addWidget(nameLable)
            layout.setStretchFactor(self.vectorDisp[index],4)
            dispWidget.setLayout(layout)
            self.vectorDispWiget.append(dispWidget)
            self.previewLayout.addWidget(dispWidget, row, col)


    def BuildDsip(self):
        if not self.cfgDispWidthEdit.text().isnumeric() or not self.cfgDispHeightEdit.text().isnumeric():
            return
        cfg_width = int(self.cfgDispWidthEdit.text())
        cfg_height = int(self.cfgDispHeightEdit.text())
        self.nowDispW = cfg_width
        self.nowDispH = cfg_height
        num_cfg = cfg_width*cfg_height
        num_now = len(self.vectorDisp)
        if(num_cfg > num_now):
            for index in range(num_cfg - num_now):
                w = gl.GLViewWidget()
                w.setCameraPosition(distance=40)
                w.setBackgroundColor(pg.mkColor(150, 150, 150, 0.6))
                self.vectorDisp.append(w)
                nameLable = QtWidgets.QLabel("default")
                nameLable.setAlignment(QtCore.Qt.AlignCenter)
                self.vectorDispName.append(nameLable)
                dispWidget = QtWidgets.QWidget()
                layout = QtWidgets.QVBoxLayout()
                layout.addWidget(w)
                layout.addWidget(nameLable)
                layout.setStretchFactor(w, 4)
                dispWidget.setLayout(layout)
                self.vectorDispWiget.append(dispWidget)
            for index in range(len(self.vectorDisp)):
                assert len(self.vectorDisp) == len(self.vectorDispWiget)
                assert len(self.vectorDisp) == len(self.vectorDispName)
                col = index % self.nowDispW
                row = int(index / self.nowDispW)
                self.previewLayout.addWidget(self.vectorDispWiget[index], row, col)
        elif (num_cfg < num_now):
            for index in range(num_now - num_cfg):
                assert len(self.vectorDisp) == len(self.vectorDispWiget)
                assert len(self.vectorDisp) == len(self.vectorDispName)
                self.vectorDisp.pop()
                self.vectorDispName.pop()
                dispWidget = self.vectorDispWiget.pop()
                self.previewLayout.removeWidget(dispWidget)
                dispWidget.close()
            for index in range(len(self.vectorDisp)):
                assert len(self.vectorDisp) == len(self.vectorDispWiget)
                assert len(self.vectorDisp) == len(self.vectorDispName)
                col = index % self.nowDispW
                row = int(index / self.nowDispW)
                self.previewLayout.addWidget(self.vectorDispWiget[index], row, col)


    def drawData(self):
        dataType = self.formatCombo.currentIndex()
        skip_data = int(self.skipData.text())
        skip_frame = int(self.skipFrame.text())
        keyFrameCount = int(self.keyFrameCount.text())
        if(skip_data <=0 or skip_frame <=0 or keyFrameCount<=0):
            return
        data_ = np.loadtxt(strip_first_col(self.filename))
        now_row = int(self.inserIndex_Row.text())
        now_col = int(self.inserIndex_Col.text())
        now_widget: gl.GLViewWidget = self.vectorDisp[now_row * self.nowDispW + now_col]
        now_widget.clear()
        worldFame = drawCoordinateFrameItem(np.array([0,0,0]), np.array([0,0,0]), is_keyframe=True,
                                        id='world_frame',
                                        parent=now_widget,
                                        axis_width=10)
        worldFame.setSize(5,5,5)
        now_widget.addItem(worldFame)
        name = self.filename.split('/')[-1]
        total_num = len(data_)
        disp_num = int(total_num / skip_data+0.5)
        addInfo = ' Total:{}, Disp:{}, Frame:{}, KeyFrame:{}'.format(total_num, disp_num,
                                                                  int(disp_num / skip_frame+0.5),
                                                                  int(disp_num / keyFrameCount+0.5))

        if(dataType == 0 or dataType==1):
            tx_index = 3
            position_ = data_[::skip_data, [tx_index, tx_index + 4, tx_index + 8]]
            RWi_ = data_[::skip_data, [0, 1, 2, 4, 5, 6, 8, 9, 10]]
            plt = gl.GLLinePlotItem(pos=position_, color=pg.mkColor(0, 255, 0), width=2, antialias=True)
            now_widget.addItem(plt)
            gz = gl.GLGridItem()
            mean_ = np.mean(position_, axis=0)
            size_ = ptp(position_, axis=0)
            add_size = 10
            gz.setSize(size_[0] + add_size, size_[1] + add_size, 10)
            gz.translate(mean_[0], mean_[1], 0)
            now_widget.addItem(gz)
            index = 0
            addInfo += ' ,Delta_Z:{}m'.format(size_[2])
            self.vectorDispName[now_row * self.nowDispW + now_col].setText(name + addInfo)
            for r, trans in zip(RWi_, position_):
                r = r.reshape(3, 3)
                if index % skip_frame ==0:
                    if index % keyFrameCount == 0 and index !=0:
                        frame = drawCoordinateFrameItem(rotationMatrixToEulerAngles(r), trans, is_keyframe=True,
                                                        id=index,
                                                        parent=now_widget,
                                                        axis_width=5)
                    elif index==0 and not np.allclose(trans,[0,0,0]):
                        frame = drawCoordinateFrameItem(r, trans, is_keyframe=True,
                                                        id='Origin',
                                                        parent=now_widget,
                                                        axis_width=10)
                    else:
                        frame = drawCoordinateFrameItem(rotationMatrixToEulerAngles(r), trans, id=index,
                                                        parent=now_widget,
                                                        axis_width=5)
                    frame.setSize(2, 2, 2)
                    now_widget.addItem(frame)
                index += 1
        elif(dataType == 2):
            pose_index_start = 4
            position_raw = data_[::skip_data, [pose_index_start, pose_index_start + 1, pose_index_start + 2]]
            quaterntions_raw = data_[::skip_data, [3, 0, 1, 2]]
            rpy_raw = [tf.euler_from_quaternion(quater) for quater in quaterntions_raw]
            plt = gl.GLLinePlotItem(pos=position_raw, color=pg.mkColor(0, 255, 0), width=2, antialias=True)
            now_widget.addItem(plt)
            gz = gl.GLGridItem()
            mean_ = np.mean(position_raw, axis=0)
            size_ = ptp(position_raw, axis=0)
            add_size = 10
            gz.setSize(size_[0] + add_size, size_[1] + add_size, 10)
            gz.translate(mean_[0], mean_[1], 0)
            now_widget.addItem(gz)
            index = 0
            addInfo += ' ,Delta_Z:{}m'.format(size_[2])
            self.vectorDispName[now_row * self.nowDispW + now_col].setText(name + addInfo)
            for r, trans in zip(rpy_raw, position_raw):
                if index % skip_frame == 0:
                    if index % keyFrameCount == 0 and index !=0:
                        frame = drawCoordinateFrameItem(r, trans, is_keyframe=True,
                                                        id=index,
                                                        parent=now_widget,
                                                        axis_width=5)
                    elif index==0 and not np.allclose(trans,[0,0,0]):
                        frame = drawCoordinateFrameItem(r, trans, is_keyframe=True,
                                                        id='Origin',
                                                        parent=now_widget,
                                                        axis_width=10)
                    else:
                        frame = drawCoordinateFrameItem(r, trans, id=index,
                                                        parent=now_widget,
                                                        axis_width=5)
                    frame.setSize(2, 2, 2)
                    now_widget.addItem(frame)
                index += 1

    def openFile(self):
        fileName_choose, filetype = QtWidgets.QFileDialog.getOpenFileName(self,"选取轨迹文件",os.getcwd(),"Text Files(*.txt);;All Files(*)")
        if fileName_choose== "":
            return
        else:
            self.filenameEdit.setText(fileName_choose)
            self.filename = fileName_choose


if __name__ == '__main__':
    app = pg.mkQApp("TrajectoryViewer")
    mainWin = TrajectoryViewer()
    mainWin.setWindowIcon(QtGui.QIcon(":/icons/Black Board.png"))
    mainWin.setMinimumSize(1200, 800)
    mainWin.show()
    pg.exec()
