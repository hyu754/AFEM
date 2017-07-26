#pragma once



#include <opencv2/viz/vizcore.hpp>
#include <opencv2/viz/widget_accessor.hpp>
#include <iostream>


#include <vtkCellArray.h>
#include <vtkHexagonalPrism.h>
#include <vtkHexahedron.h>


/*
TODO: REMOVE DUPLICATED HEADERS
*/


#include <vtkIdList.h>
#include <vtkActor.h>
#include <vtkPolyLine.h>
#include <vtkPolyData.h>
#include <vtkPolyData.h>
#include <vtkPolyhedron.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkProp.h>
#include <vtkPentagonalPrism.h>
#include <vtkPolyhedron.h>
#include <vtkPoints.h>
#include <vtkPyramid.h>


#include <vtkSmartPointer.h>
#include <vtkLine.h>
#include <vtkCellArray.h>
#include <vtkTubeFilter.h>
#include <vtkLineSource.h>


#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkTriangle.h>
#include <vtkTetra.h>
#include <vtkVoxel.h>
#include <vtkWedge.h>
#include <vtkUnstructuredGrid.h>
#include <vtkDataSetMapper.h>
/**
Defining our own triangles and triangle lines
*/
class WTriangle : public cv::viz::Widget3D
{
public:
	WTriangle(const cv::Point3f &pt1, const cv::Point3f &pt2, const cv::Point3f &pt3, const cv::viz::Color & color = cv::viz::Color::white());

};

class WLineTriangle : public cv::viz::Widget3D
{
public:
	WLineTriangle(const cv::Point3f &pt1, const cv::Point3f &pt2, const cv::Point3f &pt3, const cv::viz::Color & color = cv::viz::Color::white());

};


class WTetraHedron : public cv::viz::Widget3D
{
public:
	WTetraHedron(const cv::Point3f &pt1, const cv::Point3f &pt2, const cv::Point3f &pt3, const cv::Point3f &pt4);

};

class WTetraHedronLine : public cv::viz::Widget3D
{
public:
	WTetraHedronLine(const cv::Point3f &pt1, const cv::Point3f &pt2, const cv::Point3f &pt3, const cv::Point3f &pt4);

};

