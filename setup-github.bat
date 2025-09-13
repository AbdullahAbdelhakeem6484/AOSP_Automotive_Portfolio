@echo off
REM AOSP Automotive Portfolio - GitHub Setup Script (Windows)
REM Author: Abdullah Abdelhakeem & Abdelrahman Mourad
REM Email: abdullah.abdelhakeem657@gmail.com, abdelrahmanmourad.am@gmail.com

echo === AOSP Automotive Portfolio - GitHub Setup ===
echo Setting up repository for GitHub push...

REM Configure Git user (if not already configured)
echo Configuring Git user information...
git config user.name "Abdullah Abdelhakeem"
git config user.email "abdullah.abdelhakeem657@gmail.com"

REM Rename branch to main
echo Setting default branch to 'main'...
git branch -m main

REM Add all files to staging
echo Adding all files to Git...
git add .

REM Create initial commit
echo Creating initial commit...
git commit -m "Initial commit: AOSP Automotive Portfolio

Complete portfolio with 8 projects across 4 skill levels:
- 2 Beginner projects (Dashboard Simulator, Vehicle State Monitor)
- 2 Intermediate projects (Climate Control, Driver Assistance)
- 2 Advanced projects (Autonomous Parking, Intelligent Infotainment)
- 2 Expert projects (Fleet Management, Security Framework)

Developed by:
- Abdullah Abdelhakeem (abdullah.abdelhakeem657@gmail.com)
- Abdelrahman Mourad (abdelrahmanmourad.am@gmail.com)

Features:
- Comprehensive READMEs with setup instructions
- Hardware integration guides
- Business value propositions
- Professional documentation
- GitHub-ready structure"

REM Add remote origin
echo Adding remote repository...
git remote add origin https://github.com/AbdullahAbdelhakeem6484/AOSP_Automotive_Portfolio.git

REM Verify remote
echo Verifying remote configuration...
git remote -v

echo.
echo === Setup Complete! ===
echo ✅ Git repository initialized
echo ✅ Branch renamed to 'main'
echo ✅ All files staged and committed
echo ✅ Remote origin configured
echo.
echo Next steps:
echo 1. Push to GitHub:
echo    git push -u origin main
echo.
echo 2. If you encounter authentication issues, use:
echo    git push -u origin main --force
echo.
echo 3. Verify on GitHub:
echo    https://github.com/AbdullahAbdelhakeem6484/AOSP_Automotive_Portfolio
echo.
echo For collaboration with Abdelrahman Mourad:
echo - Add him as a collaborator in GitHub repository settings
echo - Share repository access for joint development

pause
