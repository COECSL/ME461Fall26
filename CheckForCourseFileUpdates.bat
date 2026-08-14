@echo off
setlocal

echo ==================================================
echo Updating the local repository before merging course code
echo ==================================================

echo.
echo Pulling the latest changes for the current branch...
git pull
if errorlevel 1 goto :error

echo.
echo Fetching the latest course code from "coursecode"...
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
echo Course code update completed successfully.
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
