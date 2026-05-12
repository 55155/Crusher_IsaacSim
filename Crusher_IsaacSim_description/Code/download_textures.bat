@echo off
setlocal
set OUTDIR=C:\TEMP\Crusher_IsaacSim_description\textures
mkdir "%OUTDIR%" 2>nul

echo ============================================================
echo  PolyHaven PBR Texture Downloader (CC0)
echo  Output: %OUTDIR%
echo  Total: 10 files (~120 MB)
echo ============================================================
echo.

:: ── metal_plate_02: frame / shaft / gear / bearing ──
echo [1/10] metal_plate_02 diff...
curl -L --progress-bar -o "%OUTDIR%\metal_plate_02_diff_2k.png"   "https://dl.polyhaven.org/file/ph-assets/Textures/png/2k/metal_plate_02/metal_plate_02_diff_2k.png"

echo [2/10] metal_plate_02 roughness...
curl -L --progress-bar -o "%OUTDIR%\metal_plate_02_rough_2k.png"  "https://dl.polyhaven.org/file/ph-assets/Textures/png/2k/metal_plate_02/metal_plate_02_rough_2k.png"

echo [3/10] metal_plate_02 metallic...
curl -L --progress-bar -o "%OUTDIR%\metal_plate_02_metal_2k.png"  "https://dl.polyhaven.org/file/ph-assets/Textures/png/2k/metal_plate_02/metal_plate_02_metal_2k.png"

echo [4/10] metal_plate_02 normal (GL)...
curl -L --progress-bar -o "%OUTDIR%\metal_plate_02_nor_gl_2k.png" "https://dl.polyhaven.org/file/ph-assets/Textures/png/2k/metal_plate_02/metal_plate_02_nor_gl_2k.png"

:: ── rusty_painted_metal: motor ──
echo [5/10] rusty_painted_metal diff...
curl -L --progress-bar -o "%OUTDIR%\rusty_painted_metal_diff_2k.png"   "https://dl.polyhaven.org/file/ph-assets/Textures/png/2k/rusty_painted_metal/rusty_painted_metal_diff_2k.png"

echo [6/10] rusty_painted_metal ARM (AO+Rough+Metal)...
curl -L --progress-bar -o "%OUTDIR%\rusty_painted_metal_arm_2k.png"    "https://dl.polyhaven.org/file/ph-assets/Textures/png/2k/rusty_painted_metal/rusty_painted_metal_arm_2k.png"

echo [7/10] rusty_painted_metal normal (GL)...
curl -L --progress-bar -o "%OUTDIR%\rusty_painted_metal_nor_gl_2k.png" "https://dl.polyhaven.org/file/ph-assets/Textures/png/2k/rusty_painted_metal/rusty_painted_metal_nor_gl_2k.png"

:: ── rusty_metal_02: crusher ──
echo [8/10] rusty_metal_02 diff...
curl -L --progress-bar -o "%OUTDIR%\rusty_metal_02_diff_2k.png"   "https://dl.polyhaven.org/file/ph-assets/Textures/png/2k/rusty_metal_02/rusty_metal_02_diff_2k.png"

echo [9/10] rusty_metal_02 roughness...
curl -L --progress-bar -o "%OUTDIR%\rusty_metal_02_rough_2k.png"  "https://dl.polyhaven.org/file/ph-assets/Textures/png/2k/rusty_metal_02/rusty_metal_02_rough_2k.png"

echo [10/10] rusty_metal_02 normal (GL)...
curl -L --progress-bar -o "%OUTDIR%\rusty_metal_02_nor_gl_2k.png" "https://dl.polyhaven.org/file/ph-assets/Textures/png/2k/rusty_metal_02/rusty_metal_02_nor_gl_2k.png"

echo.
echo ============================================================
echo  Done. Files saved to:
echo  %OUTDIR%
echo ============================================================
pause
