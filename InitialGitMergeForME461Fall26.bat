@echo off
setlocal

echo ==================================================
echo Adding the course repository as remote "coursecode"
echo ==================================================

git remote get-url coursecode >nul 2>&1
if %errorlevel%==0 (
    echo Remote "coursecode" already exists.
    echo Updating its URL to:
    echo https://github.com/COECSL/ME461Fall26.git
    git remote set-url coursecode https://github.com/COECSL/ME461Fall26.git
) else (
    echo Creating remote "coursecode"...
    git remote add coursecode https://github.com/COECSL/ME461Fall26.git
)

if errorlevel 1 goto :error

echo.
echo Fetching the latest course code...
git fetch coursecode
if errorlevel 1 goto :error

echo.
echo Merging coursecode/main into the current branch...
git merge coursecode/main -m "get course code"
if errorlevel 1 goto :error

echo.
echo Pushing the updated main branch to origin...
git push origin main
if errorlevel 1 goto :error

echo.
echo ==================================================
echo Course code setup and merge completed successfully.
echo ==================================================
pause
exit /b 0

:error
echo.
echo ==================================================
echo An error occurred. Review the Git output above.
echo The script has stopped without running later steps.
echo ==================================================
pause
exit /b 1
