//
// Created by 91418 on 2023/7/5.
//

#include "Multi_Process.h"
#include <processthreadsapi.h>
PROCESS_INFORMATION Multi_Process::safety_monitor_build() {
    STARTUPINFO si;
    PROCESS_INFORMATION pi;
    ZeroMemory(&si, sizeof(si));
    si.cb = sizeof(si);
    ZeroMemory(&pi, sizeof(pi));
    // Start the child process.
    if (!CreateProcess(NULL,                                                                 // No module name (use command line)
                       safety_program.data(),                                                // Command line
                       NULL,                                                                 // Process handle not inheritable
                       NULL,                                                                 // Thread handle not inheritable
                       FALSE,                                                                // Set handle inheritance to FALSE
                       NORMAL_PRIORITY_CLASS | CREATE_NEW_CONSOLE | CREATE_NEW_PROCESS_GROUP,//
                       NULL,                                                                 // Use parent's environment block
                       NULL,                                                                 // Use parent's starting directory
                       &si,                                                                  // Pointer to STARTUPINFO structure
                       &pi)                                                                  // Pointer to PROCESS_INFORMATION structure
    ) {
        printf("CreateProcess failed (%d).\n", GetLastError());
        return pi;
    }

//    processDelete(pi);
    return pi;
}
int Multi_Process::processDelete(PROCESS_INFORMATION &pi) {
    return (TerminateProcess(pi.hProcess,0));
}
PROCESS_INFORMATION Multi_Process::safety_monitor_build(const string& name) {
    STARTUPINFO si;
    PROCESS_INFORMATION pi;
    ZeroMemory(&si, sizeof(si));
    si.cb = sizeof(si);
    ZeroMemory(&pi, sizeof(pi));
    cout<<"start build safety process"<<endl;
    // Start the child process.
    if (!CreateProcess(NULL,                                                                 // No module name (use command line)
                       (char*)name.data(),                                                     // Command line
                       NULL,                                                                 // Process handle not inheritable
                       NULL,                                                                 // Thread handle not inheritable
                       FALSE,                                                                // Set handle inheritance to FALSE
                       CREATE_NEW_PROCESS_GROUP,                                                  // or  choose  CREATE_NEW_CONSOLE
                       NULL,                                                                 // Use parent's environment block
                       NULL,                                                                 // Use parent's starting directory
                       &si,                                                                  // Pointer to STARTUPINFO structure
                       &pi)                                                                  // Pointer to PROCESS_INFORMATION structure
    ) {
        cout<<"CreateProcess failed! "<<GetLastError()<<endl;
        return pi;
    }
    cout<<"SAFETY PROCESS START!"<<endl;
    // Close process and thread handles.
    return pi;
}
int Multi_Process::data_monitor(const string& name,HANDLE&handle_read) {

    SetConsoleOutputCP(CP_UTF8);
    cout<<"创建监视进程！"<<endl;
    PROCESS_INFORMATION pi;
    ZeroMemory(&pi,sizeof(PROCESS_INFORMATION));
    STARTUPINFO si;
    ZeroMemory(&si,sizeof(STARTUPINFO));
    si.cb=sizeof(STARTUPINFO);
    si.hStdInput=handle_read;
    si.dwFlags|=STARTF_USESTDHANDLES;
    bool ret = CreateProcess(NULL,
                             (char*)name.data(),
                             NULL,
                             NULL,
                             TRUE,
                             CREATE_NEW_CONSOLE|CREATE_NEW_PROCESS_GROUP,
                             NULL,
                             NULL,
                             &si,
                             &pi);
    if(!ret){
        cout<<"创建monitor 进程失败！"<<endl;
        processDelete(pi);
        return -1;
    }
    else{
        cout<<"monitor进程创建成功！ "<<endl;
//        processDelete(pi);
    }
    // Close process and thread handles.
    CloseHandle(pi.hProcess);
    CloseHandle(pi.hThread);
    return 0;
}
