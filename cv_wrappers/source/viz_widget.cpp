
#include "viz_widget.h"


/**
* @function WTriangle::WTriangle
*/
WTriangle::WTriangle(const cv::Point3f &pt1, const cv::Point3f &pt2, const cv::Point3f &pt3, const cv::viz::Color & color)
{
	// Create a triangle
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	points->InsertNextPoint(pt1.x, pt1.y, pt1.z);
	points->InsertNextPoint(pt2.x, pt2.y, pt2.z);
	points->InsertNextPoint(pt3.x, pt3.y, pt3.z);
	points->InsertNextPoint(pt1.x, pt1.y, pt1.z);

	vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New();
	polyLine->GetPointIds()->SetNumberOfIds(4);
	for (unsigned int i = 0; i < 4; i++)
	{
		polyLine->GetPointIds()->SetId(i, i);
	}


	vtkSmartPointer<vtkTriangle> triangle = vtkSmartPointer<vtkTriangle>::New();
	triangle->GetPointIds()->SetId(0, 0);
	triangle->GetPointIds()->SetId(1, 1);
	triangle->GetPointIds()->SetId(2, 2);


	vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
	cells->InsertNextCell(triangle);


	vtkSmartPointer<vtkCellArray> cells2 = vtkSmartPointer<vtkCellArray>::New();
	cells2->InsertNextCell(polyLine);

	vtkSmartPointer<vtkActor> lineActor = vtkSmartPointer<vtkActor>::New();
	vtkSmartPointer<vtkPolyData> polyDataline = vtkSmartPointer<vtkPolyData>::New();
	polyDataline->SetLines(cells2);
	polyDataline->SetPoints(points);
	vtkSmartPointer<vtkPolyDataMapper> lineMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	lineMapper->SetInputData(polyDataline);
	lineActor->SetMapper(lineMapper);
	lineActor->GetProperty()->SetColor(0, 0, 0);

	// Create a polydata object
	vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();

	// Add the geometry and topology to the polydata
	polyData->SetPoints(points);
	polyData->SetPolys(cells);



	// Create mapper and actor
	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
#if VTK_MAJOR_VERSION <= 5
	mapper->SetInput(polyData);
#else
	mapper->SetInputData(polyData);
#endif

	vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);

	// Store this actor in the widget in order that visualizer can access it

	cv::viz::WidgetAccessor::setProp(*this, lineActor);
	cv::viz::WidgetAccessor::setProp(*this, actor);

	// Set the color of the widget. This has to be called after WidgetAccessor.
	//setColor(color);
}

WLineTriangle::WLineTriangle(const cv::Point3f &pt1, const cv::Point3f &pt2, const cv::Point3f &pt3, const cv::viz::Color & color)
{
	// Create a triangle
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	points->InsertNextPoint(pt1.x, pt1.y, pt1.z);
	points->InsertNextPoint(pt2.x, pt2.y, pt2.z);
	points->InsertNextPoint(pt3.x, pt3.y, pt3.z);
	points->InsertNextPoint(pt1.x, pt1.y, pt1.z);

	vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New();
	polyLine->GetPointIds()->SetNumberOfIds(4);
	for (unsigned int i = 0; i < 4; i++)
	{
		polyLine->GetPointIds()->SetId(i, i);
	}


	vtkSmartPointer<vtkTriangle> triangle = vtkSmartPointer<vtkTriangle>::New();
	triangle->GetPointIds()->SetId(0, 0);
	triangle->GetPointIds()->SetId(1, 1);
	triangle->GetPointIds()->SetId(2, 2);


	vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
	cells->InsertNextCell(triangle);


	vtkSmartPointer<vtkCellArray> cells2 = vtkSmartPointer<vtkCellArray>::New();
	cells2->InsertNextCell(polyLine);

	vtkSmartPointer<vtkActor> lineActor = vtkSmartPointer<vtkActor>::New();
	vtkSmartPointer<vtkPolyData> polyDataline = vtkSmartPointer<vtkPolyData>::New();
	polyDataline->SetLines(cells2);
	polyDataline->SetPoints(points);
	vtkSmartPointer<vtkPolyDataMapper> lineMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	lineMapper->SetInputData(polyDataline);
	lineActor->SetMapper(lineMapper);
	lineActor->GetProperty()->SetColor(0, 0, 0);

	// Create a polydata object
	vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();

	// Add the geometry and topology to the polydata
	polyData->SetPoints(points);
	polyData->SetPolys(cells);



	// Create mapper and actor
	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
#if VTK_MAJOR_VERSION <= 5
	mapper->SetInput(polyData);
#else
	mapper->SetInputData(polyData);
#endif

	vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);

	// Store this actor in the widget in order that visualizer can access it
	

	cv::viz::WidgetAccessor::setProp(*this, actor);
	cv::viz::WidgetAccessor::setProp(*this, lineActor);
	// Set the color of the widget. This has to be called after WidgetAccessor.
	//setColor(color);
}


WTetraHedron::WTetraHedron(const cv::Point3f &pt1, const cv::Point3f &pt2, const cv::Point3f &pt3, const cv::Point3f &pt4){

	// Make a tetrahedron.
	int numberOfVertices = 4;

	vtkSmartPointer< vtkPoints > points =
		vtkSmartPointer< vtkPoints > ::New();

	points->InsertNextPoint(pt1.x, pt1.y, pt1.z);
	points->InsertNextPoint(pt2.x, pt2.y, pt2.z);
	points->InsertNextPoint(pt3.x, pt3.y, pt3.z);
	points->InsertNextPoint(pt4.x, pt4.y, pt4.z);

	vtkSmartPointer<vtkTetra> tetra =
		vtkSmartPointer<vtkTetra>::New();
	for (int i = 0; i < numberOfVertices; ++i)
	{
		tetra->GetPointIds()->SetId(i, i);
	}

	vtkSmartPointer<vtkCellArray> cellArray = 		vtkSmartPointer<vtkCellArray>::New();
	cellArray->InsertNextCell(tetra);

	vtkSmartPointer<vtkUnstructuredGrid> unstructuredGrid =		vtkSmartPointer<vtkUnstructuredGrid>::New();

	unstructuredGrid->SetPoints(points);
	unstructuredGrid->SetCells(VTK_TETRA, cellArray);

	// Create mapper and actor
	vtkSmartPointer<vtkDataSetMapper> mapper = vtkSmartPointer<vtkDataSetMapper>::New();
#if VTK_MAJOR_VERSION <= 5
	mapper->SetInput(polyData);
#else
	mapper->SetInputData(unstructuredGrid);
#endif

	vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
	actor->GetProperty()->SetOpacity(0.5);
	actor->SetMapper(mapper);

	cv::viz::WidgetAccessor::setProp(*this, actor);


}



WTetraHedronLine::WTetraHedronLine(const cv::Point3f &pt1, const cv::Point3f &pt2, const cv::Point3f &pt3, const cv::Point3f &pt4)
{
	// Create a triangle
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	points->InsertNextPoint(pt1.x, pt1.y, pt1.z);
	points->InsertNextPoint(pt2.x, pt2.y, pt2.z);
	points->InsertNextPoint(pt3.x, pt3.y, pt3.z);
	points->InsertNextPoint(pt4.x, pt4.y, pt4.z);
	//points->InsertNextPoint(pt1.x, pt1.y, pt1.z);
	//points->InsertNextPoint(pt4.x, pt4.y, pt4.z);
	//points->InsertNextPoint(pt2.x, pt2.y, pt2.z);

	//points->InsertNextPoint(pt2.x, pt2.y, pt2.z);
	//points->InsertNextPoint(pt4.x, pt4.y, pt4.z);
	//points->InsertNextPoint(pt3.x, pt3.y, pt3.z);

	//points->InsertNextPoint(pt1.x, pt1.y, pt1.z);
	//points->InsertNextPoint(pt3.x, pt3.y, pt3.z);
	//points->InsertNextPoint(pt4.x, pt4.y, pt4.z);

	vtkSmartPointer<vtkPolyLine> polyLine1 = vtkSmartPointer<vtkPolyLine>::New();
	vtkSmartPointer<vtkCellArray> cells2 = vtkSmartPointer<vtkCellArray>::New();


	//vtkSmartPointer<vtkPolyLine> polyLine1 = vtkSmartPointer<vtkPolyLine>::New();
	polyLine1->GetPointIds()->SetNumberOfIds(4);
	polyLine1->GetPointIds()->SetId(0, 0);
	polyLine1->GetPointIds()->SetId(1, 1);
	polyLine1->GetPointIds()->SetId(2, 2);

	vtkSmartPointer<vtkPolyLine> polyLine2 = vtkSmartPointer<vtkPolyLine>::New();
	polyLine2->GetPointIds()->SetNumberOfIds(4);
	polyLine2->GetPointIds()->SetId(0, 0);
	polyLine2->GetPointIds()->SetId(3, 3);
	polyLine2->GetPointIds()->SetId(1, 1);

	vtkSmartPointer<vtkPolyLine> polyLine3 = vtkSmartPointer<vtkPolyLine>::New();
	polyLine3->GetPointIds()->SetNumberOfIds(4);
	polyLine3->GetPointIds()->SetId(1, 1);
	polyLine3->GetPointIds()->SetId(3, 3);
	polyLine3->GetPointIds()->SetId(2, 2);

	vtkSmartPointer<vtkPolyLine> polyLine4 = vtkSmartPointer<vtkPolyLine>::New();
	polyLine3->GetPointIds()->SetNumberOfIds(4);
	polyLine3->GetPointIds()->SetId(0, 0);
	polyLine3->GetPointIds()->SetId(2, 2);
	polyLine3->GetPointIds()->SetId(3, 3);




	cells2->InsertNextCell(polyLine1);
	cells2->InsertNextCell(polyLine2);
	cells2->InsertNextCell(polyLine3);
	cells2->InsertNextCell(polyLine4);





	


	vtkSmartPointer<vtkActor> lineActor = vtkSmartPointer<vtkActor>::New();
	vtkSmartPointer<vtkPolyData> polyDataline = vtkSmartPointer<vtkPolyData>::New();
	polyDataline->SetLines(cells2);
	polyDataline->SetPoints(points);
	vtkSmartPointer<vtkPolyDataMapper> lineMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	lineMapper->SetInputData(polyDataline);
	lineActor->SetMapper(lineMapper);
	lineActor->GetProperty()->SetColor(0, 0, 0);




	// Store this actor in the widget in order that visualizer can access it



	cv::viz::WidgetAccessor::setProp(*this, lineActor);
	// Set the color of the widget. This has to be called after WidgetAccessor.
	//setColor(color);
}
