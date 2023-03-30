Set WshShell = CreateObject("WScript.Shell") 
WshShell.Run chr(34) & "C:\dev_ws\src\custom_gui\executable\custom_gui.bat" & Chr(34), 0
Set WshShell = Nothing