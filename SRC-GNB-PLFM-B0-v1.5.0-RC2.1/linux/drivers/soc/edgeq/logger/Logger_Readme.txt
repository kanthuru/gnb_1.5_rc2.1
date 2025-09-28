Logger Readme to add more log buffer sources
============================================

(A) Relevant files for User Space Logger Daemon
------------------------------------------------
raptor2_logger.h: common header file between user and Kernel space
raptor2_su_common.h: common header file between user and Kernel space
raptor2_userlog.h: header file for user space loger daemon
raptor2_userlog.c: source file for user space logger - multi-thread support (presently inactive)
raptor2_userlog_nt.c: source file for user space logger - single process (presently Active)

(B) Steps to modify User space Logger daemon to support new log source
-----------------------------------------------------------------------
File: raptor2_logger.h
- Define log buffer size and meta-data format as defined for L1 log buffers.
- Define L2 device name as visible to Linux user space for "mmap"ping L2 log buffer memory.

File: raptor2_userlog.h
- Define file name for new log file to be generated.
- Define Log message size, Start string, message format in raptor2_userlog.h
- Define file handle for log file, bytecount, mmap file handle etc. in logfinfo structure.

File: raptor2_userlog_nt.c
- Modify function "main" to "mmap" log buffers for the log source to be added.
- Modify function "open_mmap_logfiles" to open and mmap the new log file and stash away the descriptor information in logfinfo.
- Modify function "ppbuf_reader" to read from the new log buffer source and write to the "mmap"ped new log file.

(C) Relevant files for Linux Kernel Module Logger
--------------------------------------------------
raptor2_logger.h: Defines device name for mmap, message string meta-data. Physical memory address for log buffers. Common user and Kernel space
raptor2_su_common.h: common header file between user and Kernel space
raptor2_logger.c: Source file with entry points for open, mmap and close

(D) Steps to modify/write Linux Kernel Module to support new log source
-----------------------------------------------------------------------
****Note****
These kernel module files should be used as example only. You may need to write your own module to mmap the device memory.
****Note****

File: raptor2_logger.h
- Define log buffer size and meta-data format as defined for L1 log buffers.
- Define L2 device name as visible to Linux user space for "mmap"ping L2 log buffer memory.

File: raptor2_logger.c
- Create new fops for the new log device along the lines of r2log_device_ops and implement open, release, mmap.
- Modify or write own version of "raptor2_logger_init" to register log device name and device ops so as to get major number allocated.
- Implement "open" entry point along the lines of "r2log_device_op_open" which is invoked when user space program calls open on the device name.
- Implement "mmap" entry point along the lines of "r2log_device_op_mmap" which is invoked when user space program calls mmap on the device name.
- Implement "release" entry point along the lines of "r2log_device_op_release" which is invoked when user space program calls close on the device name.
- Define vm_ops structure along the lines of "r2log_device_vm_ops" and define the open, close and fault entry points.
- Implement "fault" entry point along the lines of "r2log_device_vma_fault" so that request for relevant memory page from user-space is fulfilled by this function

