"""Render offscreen con VTK (z-buffer real) para piezas de build123d o STL.

Uso:
    from render_vtk import render
    render([(shape_or_stl_path, "#e0762c", 1.0), ...], "out.png",
           view=(0.46, -0.73, 0.5), size=(1800, 1200))
"""
from pathlib import Path
import tempfile

import vtk


def _hex(c):
    c = c.lstrip("#")
    return tuple(int(c[i:i + 2], 16) / 255 for i in (0, 2, 4))


def _polydata(obj, tol=0.05, ang=0.15):
    if isinstance(obj, (str, Path)):
        path = str(obj)
    else:  # build123d Shape
        from build123d import export_stl
        path = tempfile.mktemp(suffix=".stl")
        export_stl(obj, path, tolerance=tol, angular_tolerance=ang)
    reader = vtk.vtkSTLReader()
    reader.SetFileName(path)
    clean = vtk.vtkCleanPolyData()
    clean.SetInputConnection(reader.GetOutputPort())
    clean.SetTolerance(1e-6)
    normals = vtk.vtkPolyDataNormals()
    normals.SetInputConnection(clean.GetOutputPort())
    normals.SetFeatureAngle(35)
    normals.SplittingOn()
    normals.ConsistencyOn()
    normals.AutoOrientNormalsOn()
    normals.Update()
    return normals.GetOutput()


def render(items, out_png, view=(0.46, -0.73, 0.5), up=(0, 0, 1), size=(1800, 1200),
           zoom=1.0, bg_top="#fbfaf7", bg_bottom="#e6e1d8", edges=True, focus=None, parallel=False):
    """items: lista de (shape | ruta STL, color hex, opacidad[, dibujar_aristas])."""
    ren = vtk.vtkRenderer()
    ren.GradientBackgroundOn()
    ren.SetBackground(*_hex(bg_bottom))
    ren.SetBackground2(*_hex(bg_top))
    for it in items:
        obj, color, opacity = it[:3]
        draw_edges = it[3] if len(it) > 3 else edges
        pd = _polydata(obj)
        m = vtk.vtkPolyDataMapper()
        m.SetInputData(pd)
        a = vtk.vtkActor()
        a.SetMapper(m)
        p = a.GetProperty()
        p.SetColor(*_hex(color))
        p.SetOpacity(opacity)
        p.SetInterpolationToPhong()
        p.SetSpecular(0.25)
        p.SetSpecularPower(30)
        p.SetAmbient(0.18)
        p.SetDiffuse(0.85)
        ren.AddActor(a)
        if draw_edges and opacity > 0.5:
            fe = vtk.vtkFeatureEdges()
            fe.SetInputData(pd)
            fe.BoundaryEdgesOn()
            fe.FeatureEdgesOn()
            fe.SetFeatureAngle(35)
            fe.ManifoldEdgesOff()
            fe.NonManifoldEdgesOff()
            em = vtk.vtkPolyDataMapper()
            em.SetInputConnection(fe.GetOutputPort())
            em.ScalarVisibilityOff()
            em.SetResolveCoincidentTopologyToPolygonOffset()
            ea = vtk.vtkActor()
            ea.SetMapper(em)
            ea.GetProperty().SetColor(0.12, 0.10, 0.09)
            ea.GetProperty().SetLineWidth(1.4)
            ea.GetProperty().SetOpacity(0.55)
            ren.AddActor(ea)

    light = vtk.vtkLightKit()
    light.SetKeyLightIntensity(0.9)
    light.AddLightsToRenderer(ren)

    cam = ren.GetActiveCamera()
    ren.ResetCamera()
    fp = focus if focus is not None else cam.GetFocalPoint()
    dist = cam.GetDistance()
    cam.SetFocalPoint(*fp)
    cam.SetPosition(*(fp[i] + view[i] / sum(v * v for v in view) ** 0.5 * dist for i in range(3)))
    cam.SetViewUp(*up)
    if parallel:
        cam.ParallelProjectionOn()
    ren.ResetCamera()
    cam.SetFocalPoint(*fp) if focus is not None else None
    cam.Zoom(zoom)
    ren.ResetCameraClippingRange()

    win = vtk.vtkRenderWindow()
    win.SetOffScreenRendering(1)
    win.SetMultiSamples(8)
    win.AddRenderer(ren)
    win.SetSize(*size)
    win.Render()

    w2i = vtk.vtkWindowToImageFilter()
    w2i.SetInput(win)
    w2i.SetScale(1)
    w2i.ReadFrontBufferOff()
    w2i.Update()
    wr = vtk.vtkPNGWriter()
    wr.SetFileName(str(out_png))
    wr.SetInputConnection(w2i.GetOutputPort())
    wr.Write()
    win.Finalize()
    return out_png


if __name__ == "__main__":
    import sys
    render([(sys.argv[1], "#e0762c", 1.0)], sys.argv[2])
