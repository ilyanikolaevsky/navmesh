#include "framework.h"
#include "resource.h"

#include "point.h"
#include "segment.h"
#include "polygon.h"
#include "path_finder.h"

#include <windows.h>
#include <windowsx.h>
#include <uxtheme.h>
#include <objidl.h>
#include <gdiplus.h>

#define _USE_MATH_DEFINES
#include <math.h>

#include <chrono>
#include <vector>
#include <sstream>

#define MAX_LOADSTRING 100

#pragma comment (lib,"Gdiplus.lib")
#pragma comment (lib, "uxtheme.lib")

// Global Variables:
HINSTANCE hInst;                                // current instance
WCHAR szTitle[MAX_LOADSTRING];                  // The title bar text
WCHAR szWindowClass[MAX_LOADSTRING];            // the main window class name
ULONG_PTR  gdiplusToken;                        // GDI context.


enum class ClickMode {
	kNone,
	kAddPolygon,
	kDeletePolygon,
	kMoveSource,
	kMoveDestination
};

// Current interaction state.
ClickMode click_mode = ClickMode::kNone;

// Debug parameters, controlled from the menu.
bool draw_edges = false;
bool inflate = true;

// Currently constructed polygon.
// Not added to the map yet.
// Is valid only if |click_mode| is kAddPolygon.
NavMesh::Polygon cur_polygon;

// All polygons on the map.
std::vector<NavMesh::Polygon> polygons;
// Indicates if there was any change to the |polygons|
// and map has to be fully updated in PathFinder.
bool polygons_changed = true;

// Current coordinates of source and destination nodes.
NavMesh::Point source_coordinates(30, 300);
NavMesh::Point dest_coordinates(1000, 300);

// Current cursor position, updated continuously.
NavMesh::Point cursor_position;


NavMesh::PathFinder path_finder;

void GeneratePolygons() {
	const int N = 100;
	const int K = 10;
	polygons.clear();
	for (int i = 0; i < N; i++) {
		NavMesh::Polygon p;
		int x = 30 + rand() % 1000;
		int y = 30 + rand() % 600;
		for (int j = 0; j < K; ++j) {
			p.AddPoint(x + rand() % 50, y + rand() % 50);
		}
		polygons.push_back(p);
	}
	polygons_changed = true;
}

void GenerateCircles() {
	const int N = 100;
	const int K = 50;
	polygons.clear();
	for (int i = 0; i < N; i++) {
		NavMesh::Polygon p;
		int x = 30 + rand() % 1000;
		int y = 30 + rand() % 600;
		for (int j = 0; j < K; ++j) {
			p.AddPoint(x + (int)cos(2 * M_PI * j / K) * 30, y + (int)sin(2 * M_PI * j / K) * 30);
		}
		polygons.push_back(p);
	}
	polygons_changed = true;
}

void GenerateGrid() {
	const int N = 100;
	polygons.clear();
	for (int i = 0; i < N; i++) {
		NavMesh::Polygon p;
		int x = 200 + i / 10 * 40;
		int y = 200 + i % 10 * 40;
		p.AddPoint(x + 10, y + 10);
		p.AddPoint(x - 10, y + 10);
		p.AddPoint(x - 10, y - 10);
		p.AddPoint(x + 10, y - 10);
		polygons.push_back(p);
	}
	polygons_changed = true;
}

Gdiplus::PointF pf(const NavMesh::Point& p) {
	return Gdiplus::PointF(static_cast<float>(p.x), static_cast<float>(p.y));
}

Gdiplus::Point pp(const NavMesh::Point& p) {
	return Gdiplus::Point(static_cast<int>(p.x), static_cast<int>(p.y));
}

void DrawSegment(const NavMesh::Segment& s, Gdiplus::Graphics& g, const Gdiplus::Pen& pen) {
	g.DrawLine(&pen, pf(s.b), pf(s.e));
}

void DrawPolygon(const NavMesh::Polygon& p, Gdiplus::Graphics& g, const Gdiplus::Pen& pen) {
	for (int i = 0; i < p.Size(); ++i) {
		DrawSegment(NavMesh::Segment(p[i], p[(i + 1) % p.Size()]), g, pen);
	}
}


const WCHAR* GetPrompt() {
	switch (click_mode) {
	case ClickMode::kAddPolygon:
		return L"Lclick - add point, Rclick - done";
	case ClickMode::kDeletePolygon:
		return L"Lclick - remove polygon, Rclick - done";
	case ClickMode::kMoveSource:
		return L"Lclick/stop dragging - fix source";
	case ClickMode::kMoveDestination:
		return L"Lclick/stop dragging - fix destination";
	default:
		return L"Choose action from the menu";
	}
}


double bench_millis = 0;

void Benchmark() {
	auto start_time = std::chrono::high_resolution_clock::now();
	const int kIterations = 500;
	for (int i = 0; i < kIterations; ++i) {
		GeneratePolygons();
		path_finder.AddPolygons(polygons, inflate ? 10 : 0);
		path_finder.AddExternalPoints({ source_coordinates, dest_coordinates });
	}
	auto geo_done_time = std::chrono::high_resolution_clock::now();
	bench_millis = (geo_done_time - start_time) / std::chrono::milliseconds(1) / (double)kIterations;

}

VOID OnPaint(HDC hdc)
{
	Gdiplus::Graphics graphics(hdc);
	Gdiplus::Pen      pen_black(Gdiplus::Color(255, 0, 0, 0));
	Gdiplus::Pen      pen_red(Gdiplus::Color(255, 255, 0, 0), 1.0);
	Gdiplus::Pen      pen_green(Gdiplus::Color(255, 0, 255, 0), 1.0);
	Gdiplus::Pen      pen_gray(Gdiplus::Color(125, 125, 125, 125), 1.0);
	Gdiplus::Pen      pen_bg(Gdiplus::Color(255, 255, 255, 255), 1.0);

	Gdiplus::SolidBrush brush_bg(Gdiplus::Color(255, 255, 255, 255));
	Gdiplus::SolidBrush brush_black(Gdiplus::Color(255, 0, 0, 0));

	// Clear the buffer.
	Gdiplus::Region clip_region;
	graphics.GetClip(&clip_region);
	graphics.FillRegion(&brush_bg, &clip_region);

	// Print the prompt.
	Gdiplus::Font myFont(L"Arial", 16);
	Gdiplus::PointF txt_origin(500.0f, 20.0f);
	graphics.DrawString(GetPrompt(), -1, &myFont, txt_origin, &brush_black);

	// Drap all polygons on the map.
	for (const auto& p : polygons) {
		bool red = (click_mode == ClickMode::kDeletePolygon) && p.IsInside(cursor_position);
		DrawPolygon(p, graphics, red ? pen_red : pen_black);
	}

	// Draw currently being constructed polygon.
	if (click_mode == ClickMode::kAddPolygon) {
		if (cur_polygon.Size() > 0) {
			DrawPolygon(cur_polygon, graphics, pen_gray);
		}

		// Draw two tangents, if possible - they would be a newly added edges.
		if (cur_polygon.Size() > 0 && !cur_polygon.IsInside(cursor_position)) {
			auto tangents = cur_polygon.GetTangentIds(cursor_position);
			if (tangents.first >= 0 && tangents.second >= 0) {
				DrawSegment(NavMesh::Segment(cursor_position, cur_polygon[tangents.first]), graphics, pen_red);
				DrawSegment(NavMesh::Segment(cursor_position, cur_polygon[tangents.second]), graphics, pen_green);
			}
		}
	}

	auto start_time = std::chrono::high_resolution_clock::now();

	// Update the map if needed.
	if (polygons_changed) {
		path_finder.AddPolygons(polygons, inflate ? 10 : 0);
		polygons_changed = false;
	}
	// Update source and destination.
	path_finder.AddExternalPoints({ source_coordinates, dest_coordinates });

	auto geo_done_time = std::chrono::high_resolution_clock::now();

	std::vector<NavMesh::Point> path = path_finder.GetPath(source_coordinates, dest_coordinates);

	auto end_time = std::chrono::high_resolution_clock::now();

	if (draw_edges) {
		auto edges = path_finder.GetEdgesForDebug();
		for (const auto& e : edges) {
			DrawSegment(e, graphics, pen_gray);
		}
		std::wstringstream ss;
		ss << edges.size() << " edges";
		graphics.DrawString(ss.str().c_str(), -1, &myFont, Gdiplus::PointF(1000.0, 10.0), &brush_black);
	}

	// Draw the path at last, to make it visible on top of
	// everything else.
	for (size_t i = 0; i + 1 < path.size(); ++i) {
		DrawSegment(NavMesh::Segment(path[i], path[i + 1]), graphics, pen_red);
	}

	// Draw endpoints as circles so they would be visible.
	graphics.DrawEllipse(&pen_green, Gdiplus::Rect((int)source_coordinates.x - 2, (int)source_coordinates.y - 4, 7, 7));
	graphics.DrawEllipse(&pen_red, Gdiplus::Rect((int)dest_coordinates.x - 2, (int)dest_coordinates.y - 4, 7, 7));


	// Time measurements.
	int64_t millis_total = (end_time - start_time) / std::chrono::milliseconds(1);
	int64_t millis_graph = (geo_done_time - start_time) / std::chrono::milliseconds(1);
	std::wstringstream ss;
	ss << "total:" << millis_total << "ms\ngeo:" << millis_graph << "ms"
		<< "\nbench: " << bench_millis << "ms";
	graphics.DrawString(ss.str().c_str(), -1, &myFont, Gdiplus::PointF(10.0, 10.0), &brush_black);
}

LRESULT CALLBACK WndProc(HWND, UINT, WPARAM, LPARAM);

// Forward declarations of functions included in this code module:
ATOM                MyRegisterClass(HINSTANCE hInstance);
BOOL                InitInstance(HINSTANCE, int);
LRESULT CALLBACK    WndProc(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK    About(HWND, UINT, WPARAM, LPARAM);

int APIENTRY wWinMain(_In_ HINSTANCE hInstance,
	_In_opt_ HINSTANCE hPrevInstance,
	_In_ LPWSTR    lpCmdLine,
	_In_ int       nCmdShow)
{
	UNREFERENCED_PARAMETER(hPrevInstance);
	UNREFERENCED_PARAMETER(lpCmdLine);

	// TODO: Place code here.

	// Initialize global strings
	LoadStringW(hInstance, IDS_APP_TITLE, szTitle, MAX_LOADSTRING);
	LoadStringW(hInstance, IDC_NAVMESH, szWindowClass, MAX_LOADSTRING);
	MyRegisterClass(hInstance);

	// Perform application initialization:
	if (!InitInstance(hInstance, nCmdShow))
	{
		return FALSE;
	}

	HACCEL hAccelTable = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDC_NAVMESH));

	MSG msg;

	// Main message loop:
	while (GetMessage(&msg, nullptr, 0, 0))
	{
		if (!TranslateAccelerator(msg.hwnd, hAccelTable, &msg))
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}
	}

	Gdiplus::GdiplusShutdown(gdiplusToken);
	return (int)msg.wParam;
}

//
//  FUNCTION: MyRegisterClass()
//
//  PURPOSE: Registers the window class.
//
ATOM MyRegisterClass(HINSTANCE hInstance)
{
	WNDCLASSEXW wcex;

	wcex.cbSize = sizeof(WNDCLASSEX);

	wcex.style = CS_HREDRAW | CS_VREDRAW;
	wcex.lpfnWndProc = WndProc;
	wcex.cbClsExtra = 0;
	wcex.cbWndExtra = 0;
	wcex.hInstance = hInstance;
	wcex.hIcon = LoadIcon(hInstance, MAKEINTRESOURCE(IDI_NAVMESH));
	wcex.hCursor = LoadCursor(nullptr, IDC_ARROW);
	wcex.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);
	wcex.lpszMenuName = MAKEINTRESOURCEW(IDC_NAVMESH);
	wcex.lpszClassName = szWindowClass;
	wcex.hIconSm = LoadIcon(wcex.hInstance, MAKEINTRESOURCE(IDI_SMALL));

	return RegisterClassExW(&wcex);
}

//
//   FUNCTION: InitInstance(HINSTANCE, int)
//
//   PURPOSE: Saves instance handle and creates main window
//
//   COMMENTS:
//
//        In this function, we save the instance handle in a global variable and
//        create and display the main program window.
//
BOOL InitInstance(HINSTANCE hInstance, int nCmdShow)
{
	hInst = hInstance; // Store instance handle in our global variable


	HWND                hWnd;
	Gdiplus::GdiplusStartupInput gdiplusStartupInput;

	// Initialize GDI+.
	Gdiplus::GdiplusStartup(&gdiplusToken, &gdiplusStartupInput, NULL);

	hWnd = CreateWindow(
		szWindowClass,   // window class name
		szTitle,  // window caption
		WS_OVERLAPPEDWINDOW,      // window style
		CW_USEDEFAULT,            // initial x position
		CW_USEDEFAULT,            // initial y position
		CW_USEDEFAULT,            // initial x size
		CW_USEDEFAULT,            // initial y size
		NULL,                     // parent window handle
		NULL,                     // window menu handle
		hInstance,                // program instance handle
		NULL);                    // creation parameters

	if (!hWnd)
	{
		return FALSE;
	}

	ShowWindow(hWnd, nCmdShow);
	UpdateWindow(hWnd);

	return TRUE;
}

//
//  FUNCTION: WndProc(HWND, UINT, WPARAM, LPARAM)
//
//  PURPOSE: Processes messages for the main window.
//
//  WM_COMMAND  - process the application menu
//  WM_PAINT    - Paint the main window
//  WM_DESTROY  - post a quit message and return
//
//
LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	switch (message)
	{
	case WM_COMMAND:
	{
		HMENU hMenu = GetMenu(hWnd);
		int wmId = LOWORD(wParam);
		// Parse the menu selections:
		switch (wmId)
		{
		case IDM_EXIT:
			DestroyWindow(hWnd);
			break;
		case ID_ADD_POLYGON:
			click_mode = click_mode == ClickMode::kAddPolygon ? ClickMode::kNone : ClickMode::kAddPolygon;
			if (click_mode == ClickMode::kAddPolygon) {
				cur_polygon.Clear();
			}
			InvalidateRect(hWnd, NULL, NULL);
			break;
		case ID_DELETE_POLYGON:
			click_mode = click_mode == ClickMode::kDeletePolygon ? ClickMode::kNone : ClickMode::kDeletePolygon;
			InvalidateRect(hWnd, NULL, NULL);
			break;
		case ID_MOVE_SOURCE:
			click_mode = click_mode == ClickMode::kMoveSource ? ClickMode::kNone : ClickMode::kMoveSource;
			InvalidateRect(hWnd, NULL, NULL);
			break;
		case ID_MOVE_DESTINATION:
			click_mode = click_mode == ClickMode::kMoveDestination ? ClickMode::kNone : ClickMode::kMoveDestination;
			InvalidateRect(hWnd, NULL, NULL);
			break;
		case ID_DEBUG_TOGGLEEDGES:
			draw_edges = !draw_edges;
			InvalidateRect(hWnd, NULL, NULL);
			break;
		case ID_DEBUG_GENERATEPOLYGONS:
			GeneratePolygons();
			InvalidateRect(hWnd, NULL, NULL);
			break;
		case ID_DEBUG_GENERATEGRID:
			GenerateGrid();
			InvalidateRect(hWnd, NULL, NULL);
			break;
		case ID_DEBUG_TOGGLEINFLATION:
			inflate = !inflate;
			polygons_changed = true;
			InvalidateRect(hWnd, NULL, NULL);
			break;
		case ID_DEBUG_GENERATECIRCLES:
			GenerateCircles();
			InvalidateRect(hWnd, NULL, NULL);
			break;
		case ID_DEBUG_BENCHMARK:
			Benchmark();
			InvalidateRect(hWnd, NULL, NULL);

		default:
			return DefWindowProc(hWnd, message, wParam, lParam);
		}
		CheckMenuItem(hMenu, ID_DEBUG_TOGGLEEDGES, draw_edges ? MF_CHECKED : MF_UNCHECKED);
		CheckMenuItem(hMenu, ID_DEBUG_TOGGLEINFLATION, inflate ? MF_CHECKED : MF_UNCHECKED);
	}
	break;
	case WM_PAINT:
	{
		PAINTSTRUCT ps;
		HDC hdc = BeginPaint(hWnd, &ps);
		RECT rc;
		GetClientRect(hWnd, &rc);
		HDC memdc;
		auto hbuff = BeginBufferedPaint(hdc, &rc, BPBF_COMPATIBLEBITMAP, NULL, &memdc);
		if (hbuff != 0) {
			OnPaint(memdc);
			EndBufferedPaint(hbuff, TRUE);
		}
		EndPaint(hWnd, &ps);
	}
	break;
	case WM_DESTROY:
		PostQuitMessage(0);
		break;
	case WM_MOUSEMOVE:
		cursor_position = NavMesh::Point(GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam));
		if (click_mode == ClickMode::kMoveDestination) {
			dest_coordinates = cursor_position;
		}
		else if (click_mode == ClickMode::kMoveSource) {
			source_coordinates = cursor_position;
		}
		if (click_mode != ClickMode::kNone) {
			// To ensure that changes are drawn.
			InvalidateRect(hWnd, NULL, NULL);
		}
		break;
	case WM_LBUTTONUP:
		if (click_mode == ClickMode::kAddPolygon) {
			cur_polygon.AddPoint(cursor_position);
		}
		else if (click_mode == ClickMode::kDeletePolygon) {
			size_t j = 0;
			for (size_t i = 0; i < polygons.size(); ++i) {
				if (!polygons[i].IsInside(cursor_position)) {
					polygons[j++] = std::move(polygons[i]);
				}
				else {
					// i-th was not copied, hence it's deleted.
					polygons_changed = true;
				}
			}
			polygons.resize(j);
		}
		else {
			// Stopped dragging source or destination.
			click_mode = ClickMode::kNone;
		}
		// Click most likelyt updated something. Redraw.
		InvalidateRect(hWnd, NULL, NULL);
		break;

	case WM_LBUTTONDOWN:
		// To allow dragging source or destination.
		if (click_mode == ClickMode::kNone) {
			if ((cursor_position - dest_coordinates).Len() < 10) {
				click_mode = ClickMode::kMoveDestination;
			}
			else if ((cursor_position - source_coordinates).Len() < 10) {
				click_mode = ClickMode::kMoveSource;
			}
		} break;

	case WM_RBUTTONUP:
		if (click_mode == ClickMode::kAddPolygon) {
			// The first right-click adds current polygon to the map.
			// The second exits adding polygons mode.
			if (cur_polygon.Size() > 2) {
				polygons.push_back(cur_polygon);
				polygons_changed = true;
				cur_polygon.Clear();
			}
			else {
				click_mode = ClickMode::kNone;
			}
		}
		else if (click_mode == ClickMode::kDeletePolygon) {
			// Just exit the mode.
			click_mode = ClickMode::kNone;
		}

		InvalidateRect(hWnd, NULL, NULL);
		break;
	default:
		return DefWindowProc(hWnd, message, wParam, lParam);
	}
	return 0;
}


// Message handler for about box.
INT_PTR CALLBACK About(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
	UNREFERENCED_PARAMETER(lParam);
	switch (message)
	{
	case WM_INITDIALOG:
		return (INT_PTR)TRUE;

	case WM_COMMAND:
		if (LOWORD(wParam) == IDOK || LOWORD(wParam) == IDCANCEL)
		{
			EndDialog(hDlg, LOWORD(wParam));
			return (INT_PTR)TRUE;
		}
		break;
	}
	return (INT_PTR)FALSE;
}