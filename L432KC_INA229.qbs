import qbs
import qbs.File
import qbs.FileInfo
import qbs.ModUtils
import qbs.TextFile

Project{

    //# cpu
    property string CPU:{
        var file = TextFile(FileInfo.joinPaths(path, 'Makefile'))
        var regex = /CPU = (\S+)/
        while(!file.atEof()){
            var result = regex.exec(file.readLine());
            if(result !== null){
                file.close()
                return result[1]
            }
        }
        return ''
    }
    //# fpu
    property string FPU:{
        var file = TextFile(FileInfo.joinPaths(path, 'Makefile'))
        var regex = /FPU = (\S+)/
        while(!file.atEof()){
            var result = regex.exec(file.readLine());
            if(result !== null){
                file.close()
                return result[1]
            }
        }
        return ''
    }
    //# float-abi
    property string FLOAT_ABI:{
        var file = TextFile(FileInfo.joinPaths(path, 'Makefile'))
        var regex = /FLOAT-ABI = (\S+)/
        while(!file.atEof()){
            var result = regex.exec(file.readLine());
            if(result !== null){
                file.close()
                return result[1]
            }
        }
        return ''
    }
    //# mcu
    // property string   MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

    CppApplication {

        condition: {
            return (qbs.toolchain.contains('gcc') && !qbs.toolchain.contains('xcode'))
        }

        type: ['application', 'bin_hex_size']

        //name: 'stm32g4xx'

        consoleApplication: true

        cpp.cLanguageVersion: 'c11'
        cpp.cxxLanguageVersion: 'c++20'

        cpp.debugInformation: true//qbs.buildVariant !== 'release'
        cpp.enableExceptions : false
        cpp.enableRtti : false
        cpp.executableSuffix: '.elf'
        cpp.optimization: qbs.buildVariant === 'release' ? 'fast' : 'none'
        cpp.positionIndependentCode: false


        cpp.includePaths:{
            var includePaths = []
            if(1){
                var file = TextFile(FileInfo.joinPaths(path, 'Makefile'))
                var regex = /-I(\S+)\s*.?/
                while(!file.atEof()){
                    var result = regex.exec(file.readLine());
                    if(result !== null)
                        includePaths.push(result[1])
                }
                file.close()
            }else{
                var exclude = [
                            'DSP',
                            'RTOS2',
                            'RTOS',
                            'NN',
                            'Template',
                            'Core_A',
                        ]
                var func = function(path){
                    var filePpaths = File.directoryEntries(path, File.Files)
                    var size = includePaths.length
                    var fl = true
                    for(var i in exclude)
                        if(path.endsWith(exclude[i]))
                            fl = false
                    for(var i in filePpaths)
                        if(fl && size === includePaths.length && (filePpaths[i].endsWith('.h')||filePpaths[i].endsWith('.hpp')))
                            includePaths.push(path)
                    var dirPaths = File.directoryEntries(path, File.Dirs | File.NoDotAndDotDot)
                    for(var i in dirPaths)
                        func(FileInfo.joinPaths(path, dirPaths[i]))
                }
                func(path)
            }
            return includePaths
        }

        cpp.defines:{
            var defines = [
                        //                        'ARDUINO=100'
                    ]
            var file = TextFile(FileInfo.joinPaths(path, 'Makefile'))
            var regex = /-D(\S+)\s*.?/
            while(!file.atEof()){
                var result = regex.exec(file.readLine());
                if(result !== null)
                    defines.push(result[1])
            }
            file.close()
            return defines
        }

        cpp.assemblerFlags: [
            CPU,
            FLOAT_ABI,
            FPU,
            '-mthumb',
        ]

        cpp.driverFlags: [
            CPU,
            FLOAT_ABI,
            FPU,
            '-mthumb',
            '-Wall',
            '-Wno-unused-parameter',
            '-fdata-sections',
            '-ffunction-sections',
            '-finline-functions',
            '-fstack-usage',
            '-specs=nano.specs',
            '-specs=nosys.specs',
            //            '-fno-inline-small-functions',//?
        ]

        cpp.cxxFlags: [
            '-Wno-volatile'
        ]

        cpp.linkerMode : 'manual'
        cpp.linkerName : 'g++'
        cpp.linkerFlags : [
            CPU,
            FLOAT_ABI,
            FPU,
            '-mthumb',
            '--specs=nano.specs',
            '--specs=nosys.specs',
            '-static',
            '-Wl,-u', '-Wl,_printf_float',
            //'-Wl,-u', '-Wl,_scanf_float',
            '-Wl,--gc-sections',
//            '-Wl,-Map='+path+'/map.map',
            '-Wl,--start-group',
            '-lc',
            '-lm',
            '-lstdc++',
            '-lsupc++',
            '-Wl,--end-group',
            //'-o @'+path+'/objects.list',
        ]


        Group {
            name: 'SRC'
            prefix: path
            //fileTags: ['c']
            files: [
                //                '/**/*.c',
                '/**/*.cpp',
                '/**/*.h',
            ]
            excludeFiles: [
                '/**/DSP/**/',
                '/**/RTOS2/**/',
                '/**/RTOS/**/',
                '/**/NN/**/',
                '/**/Template/**/',
                '/**/Core_A/**/',
                //                '/**/main.c',
                //                '/**/sysmem.c',
            ]
        }

        Group {
            name: 'C'
            prefix: path
            fileTags: ['c']
            files: ['/**/*.c']
            excludeFiles: ['/**/main.c']
            //            {
            //                var files = []
            //                var exclude = [
            //                            //                            'main.c',
            //                            //                            'sysmem.c',
            //                        ]
            //                var file = TextFile(FileInfo.joinPaths(path, 'Makefile'))
            //                var regex = /^(\S+\.c)\s*.?$/

            //                while(!file.atEof()){
            //                    var result = regex.exec(file.readLine());
            //                    if(result !== null){
            //                        var fl = true
            //                        for(var i in exclude)
            //                            if(result[1].endsWith(exclude[i]))
            //                                fl = false
            //                        if(fl)
            //                            files.push(result[1])
            //                    }
            //                }
            //                file.close()
            //                return files
            //            }
        }


        Group{
            name: 'LINKERSCRIPT'
            fileTags:  'linkerscript'
            prefix: path
            files:['/**/*.ld']
            excludeFiles: ['/**/*M.ld']
        }

        Group{
            name: 'ASM'
            fileTags:  'asm'
            prefix: path
            files:['/**/*.s']
        }

        Rule {
            inputs: ['application']
            alwaysRun: true

            Artifact {
                fileTags: ['bin_hex_size']
                //                filePath: input.baseDir + '/' + input.baseName + '.bin'
            }

            prepare: {
                var objCopyPath = 'arm-none-eabi-objcopy'
                var argsConvBin = ['-O', 'binary', input.filePath, project.path + '/' + project.name + '.bin']
                var cmdBin = new Command(objCopyPath, argsConvBin)
                cmdBin.description = 'converting to BIN: ' + FileInfo.fileName(input.filePath) + ' -> ' + input.baseName + '.bin'

                var argsConvHex = ['-O', 'ihex', input.filePath, project.path + '/'/*'E:/'*/ + project.name + '.hex']
                var cmdHex = new Command(objCopyPath, argsConvHex)
                cmdHex.description = 'converting to HEX: ' + FileInfo.fileName(input.filePath) + ' -> ' + input.baseName + '.hex'

                var objSize = 'arm-none-eabi-size'
                var argsSize = [input.filePath]
                var cmdSize = new Command(objSize, argsSize)
                cmdSize.description = 'Size: ' + FileInfo.fileName(input.filePath)
                cmdSize.stdoutFilterFunction = function(stdout){
                    var sizeOfRam = (48+16) * 1024
                    var sizeOfFlash = 256 * 1024
                    var regex = /(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+.+/g
                    var result = regex.exec(stdout);
                    stdout += '\nSize Of Ram  : ' + ((result[2] / sizeOfRam) * 100).toFixed(2) + ' %'
                    stdout += '\nSize Of Flash: ' + (((result[4] - result[2]) / sizeOfFlash) * 100).toFixed(2) + ' %'
                    return stdout
                };
                //            // Запись в nor память по qspi
                //            // Write to the nor memory by qspi
                //            var argsFlashingQspi =
                //                    [           '-f', 'board/stm32f746g-disco.cfg',
                //                     '-c', 'init',
                //                     '-c', 'reset init',
                //                     '-c', 'flash write_bank 1 ' + output.filePath + ' 0',
                //                     '-c', 'reset',
                //                     '-c', 'shutdown'
                //                    ]

                //            var cmdFlashQspi = new Command('openocd', argsFlashingQspi);
                //            cmdFlashQspi.description = 'Wrtie to the NOR QSPI'

                //            // Запись во внутреннюю память
                //            // Write to the internal memory
                //            var argsFlashingInternalFlash =
                //                    [           '-f', 'board/stm32f746g-disco.cfg',
                //                     '-c', 'init',
                //                     '-c', 'reset init',
                //                     '-c', 'flash write_image erase ' + input.filePath,
                //                     '-c', 'reset',
                //                     '-c', 'shutdown'
                //                    ]

                //            var cmdFlashInternalFlash = new Command('openocd', argsFlashingInternalFlash);
                //            cmdFlashInternalFlash.description = 'Wrtie to the internal flash'
                return [
                            cmdBin,
                            cmdHex,
                            cmdSize
                            //cmdFlashQspi,
                            //cmdFlashInternalFlash
                        ]
            }
        }
    }
}
