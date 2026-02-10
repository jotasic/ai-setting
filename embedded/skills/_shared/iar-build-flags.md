# IAR EWARM Compiler & Linker Flags Reference

## Compiler (iccarm) Key Options

### Optimization
| Option | Description |
|--------|-------------|
| `--no_optimization` | No optimization (Debug) |
| `-Ol` | Low optimization |
| `-Om` | Medium optimization |
| `-Oh` | High optimization |
| `--size` | Optimize for size (with -Oh) |
| `--speed` | Optimize for speed (with -Oh) |
| `--no_size_constraints` | Allow any size increase for speed |

### Language
| Option | Description |
|--------|-------------|
| `--c89` | C89/C90 standard |
| `--c99` | C99 standard (recommended) |
| `-e` | Enable language extensions |
| `--strict` | Strict standard compliance |
| `--require_prototypes` | Require function prototypes |

### Diagnostics
| Option | Description |
|--------|-------------|
| `--diag_suppress=PeXXX` | Suppress warning |
| `--diag_error=PeXXX` | Promote warning to error |
| `--misrac2012` | Enable MISRA-C:2012 checking |

### Code Generation
| Option | Description |
|--------|-------------|
| `--cpu=Cortex-M3` | Target CPU |
| `--fpu=None` | No FPU (Cortex-M3 has no FPU) |
| `--endian=little` | Little-endian |
| `--thumb` | Generate Thumb-2 code |

### Preprocessor Defines
```
STM32F10X_HD          /* High-density device */
USE_STDPERIPH_DRIVER  /* Enable SPL */
HSE_VALUE=8000000     /* External crystal frequency */
```

## Linker (ilinkarm) Key Options

| Option | Description |
|--------|-------------|
| `--config file.icf` | Linker configuration file |
| `--map file.map` | Generate map file |
| `--entry __iar_program_start` | Entry point |
| `--merge_duplicate_sections` | Merge identical sections |
| `--vfe` | Virtual function elimination |

## iarbuild Command Line

```bash
# Build
iarbuild <project.ewp> -build <config>

# Rebuild (clean + build)
iarbuild <project.ewp> -clean <config>
iarbuild <project.ewp> -build <config>

# Make (build only changed)
iarbuild <project.ewp> -make <config>

# Common configurations
iarbuild Project.ewp -build Release
iarbuild Project.ewp -build Debug
```

## Recommended Settings per Build Config

### Debug
```
Optimization: None
Debug info: Full
Defines: DEBUG, STM32F10X_HD, USE_STDPERIPH_DRIVER
Stack: 4KB (extra for debug)
```

### Release
```
Optimization: High/Size
Debug info: None (or minimal)
Defines: NDEBUG, STM32F10X_HD, USE_STDPERIPH_DRIVER
Stack: 2KB (calculated)
```
