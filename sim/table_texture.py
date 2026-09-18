"""Deterministic, nonrepeating printed surface for synthetic feature matching."""
from pathlib import Path
import random
import math


def create_texture(destination: Path):
    from PIL import Image, ImageDraw
    rng = random.Random(104729)
    image = Image.new("RGB",(1024,1024),(190,181,161))
    draw = ImageDraw.Draw(image)
    # 30--80 mm printed marks survive the more distant headset's sampling.
    # Random shapes and internal cutouts avoid a periodic or encoded marker grid.
    for i in range(60):
        x,y = rng.randrange(80,940),rng.randrange(80,940)
        width,height = rng.randrange(56,149),rng.randrange(61,164)
        value = rng.randrange(5,100)
        color = (value,value,value)
        if i%4 == 0:
            draw.ellipse((x-width//2,y-height//2,x+width//2,y+height//2),fill=color)
        else:
            angles = sorted(rng.uniform(0,2*math.pi) for _ in range(rng.randrange(4,9)))
            vertices = [(x+math.cos(a)*width*rng.uniform(.3,.55),
                         y+math.sin(a)*height*rng.uniform(.3,.55)) for a in angles]
            draw.polygon(vertices,fill=color)
        if i%3:
            dx,dy = rng.randrange(-15,15),rng.randrange(-15,15)
            inner = (225,216,196)
            draw.ellipse((x+dx-width//7,y+dy-height//7,x+dx+width//7,y+dy+height//7),fill=inner)
        if i%5 == 0:
            draw.line((x-width//2,y-height//2,x+width//2,y+height//2),fill=(15,15,15),width=9)
    destination.parent.mkdir(parents=True,exist_ok=True)
    image.save(destination)


def add_textured_surface(stage, texture_path: Path):
    from pxr import Gf,Sdf,UsdGeom,UsdShade
    create_texture(texture_path)
    plane = UsdGeom.Mesh.Define(stage,"/World/TablePattern")
    plane.CreatePointsAttr([(.005,-.55,-.149),(.555,-.55,-.149),(.555,-.05,-.149),(.005,-.05,-.149)])
    plane.CreateFaceVertexCountsAttr([4])
    plane.CreateFaceVertexIndicesAttr([0,1,2,3])
    plane.CreateSubdivisionSchemeAttr("none")
    plane.CreateNormalsAttr([Gf.Vec3f(0,0,1)]*4)
    uv = UsdGeom.PrimvarsAPI(plane).CreatePrimvar("st",Sdf.ValueTypeNames.TexCoord2fArray,"vertex")
    uv.Set([(0,0),(1,0),(1,1),(0,1)])
    material = UsdShade.Material.Define(stage,"/World/Looks/TablePattern")
    shader = UsdShade.Shader.Define(stage,"/World/Looks/TablePattern/Surface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("roughness",Sdf.ValueTypeNames.Float).Set(.9)
    texture = UsdShade.Shader.Define(stage,"/World/Looks/TablePattern/Texture")
    texture.CreateIdAttr("UsdUVTexture")
    texture.CreateInput("file",Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(str(texture_path.resolve())))
    texture.CreateInput("sourceColorSpace",Sdf.ValueTypeNames.Token).Set("sRGB")
    texture.CreateOutput("rgb",Sdf.ValueTypeNames.Float3)
    reader = UsdShade.Shader.Define(stage,"/World/Looks/TablePattern/UV")
    reader.CreateIdAttr("UsdPrimvarReader_float2")
    reader.CreateInput("varname",Sdf.ValueTypeNames.Token).Set("st")
    reader.CreateOutput("result",Sdf.ValueTypeNames.Float2)
    texture.CreateInput("st",Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(),"result")
    shader.CreateInput("diffuseColor",Sdf.ValueTypeNames.Color3f).ConnectToSource(texture.ConnectableAPI(),"rgb")
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),"surface")
    UsdShade.MaterialBindingAPI.Apply(plane.GetPrim()).Bind(material)
