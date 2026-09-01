@echo off
setlocal

set "ASTYLE=astyle.exe"
set "INFILE=%TEMP%\astyle_clipboard_in.c"
set "OUTFILE=%TEMP%\astyle_clipboard_out.c"

rem Copy clipboard text to a temporary C file
powershell -NoProfile -Command ^
  "$text = Get-Clipboard -Raw; [System.IO.File]::WriteAllText('%INFILE%', $text)"

rem Format the code with AStyle
"%ASTYLE%" --quiet --indent=spaces=4 < "%INFILE%" > "%OUTFILE%"

rem Put formatted code back onto the clipboard
powershell -NoProfile -Command ^
  "$text = [System.IO.File]::ReadAllText('%OUTFILE%'); Set-Clipboard -Value $text"

rem Delete temporary files
del "%INFILE%" >nul 2>&1
del "%OUTFILE%" >nul 2>&1

echo.
echo C code formatted and copied back to clipboard.
echo.
pause