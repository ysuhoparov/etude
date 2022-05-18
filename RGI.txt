.286

RGM_DATA    SEGMENT PUBLIC 'FAR_DATA'
include RGIDATA.INC
RGM_DATA    ENDS

ASSUME CS:RGI_TEXT, DS:RGM_DATA

RGI_TEXT segment

PUBLIC  _int8Entry

tabTakt dw InUStAdcI, exInt8, InIStAdcU, ULimit

Int8 PROC       ; interrupt

onTirystor:
        mov     dx,_dt
        or      _dtImage,onTyrMask
        mov     al,_dtImage
        out     dx,al
        jmp     adcOrInd

;mw2:    mov     _w2,ax
;        jmp     rmw2

offTyr:                              ; фронт синхроимпульса сети обнаружен
        mov     _lastSync,ah

;       mov     ax,_countF
;       cmp      _lastSync,0
;       jz       mw2
;       mov     _w1,ax
;rmw2:
        mov     _countF,0
        mov     dx,_dt
        and     _dtImage,offTyrMask
        mov     al,_dtImage
        out     dx,al
  ;  cmp _b1,34
  ;  jne mnkz
  ;  or     _dtImage,2
  ;  mov     al,_dtImage      ; imit KZ
  ;  out     dx,al
;mnkz:
        mov     _flag100,1
        jmp     adcOrInd        ; на определение такта прерываний
;*************************************************************************
_int8Entry:
        cli
        push    ax
        mov     al,20h
        out     20h,al          ; ready for int
        push    bx
        push    dx
        push    ds
        mov     ax,RGM_DATA
        mov     ds,ax
        mov     dx,_pi
        in      al,dx
        and     al,80h
        mov     ah,al
        xor     al,_lastSync
        jnz     offtyr
        inc     _countF
        mov     ax,_countF      ; загрузка countF - по нулю сети
        cmp     ax,_Phi
        je      onTirystor
adcOrInd:                       ; определение такта прерываний
        add     _takt,2         ; 1-Запуск АЦП напр., индикация
        mov     bx,_takt        ; 2-Ввод напр., анализ пробоя, запуск АЦП тока
        and     bx,6            ; 3-Ввод тока
        jmp     tabTakt[bx]     ; 4-Ограничение напряжения(мгновенные значения)

exCnt1: mov     _cnt,-1
        jmp     exInt8

InUStAdcI:      ; input u, Start i , check Spark
        mov     dx,0158h
        in      ax,dx
        shr     ax,1
        shr     ax,1
        shr     ax,1
        shr     ax,1
        mov     bl,al           ; bl <- u
        mov     al,51h          ; Start I
        mov     dx,0159h
        out     dx,al
        cmp     _cnt,-1
        jne     treatSpark
        cmp     bl,_uSpark
        jb      Spark
nSpark: mov     _uLast,bl
exInt8: pop     ds
        pop     dx
        pop     bx
        pop     ax
        sti
        iret

pass1:  mov     al,bl           ; bl <- u
        sub     al,_uLast       ; u - (u_n-1)
        mov     _uLast,bl
        cmp     al,-32          ; _dU  ; ? -du/dt < -15kV
        jg      exCnt1
        inc     _NumbSpark       ; число пробоев за полуволну сети < 2
        mov     ax,_index        ; Сохранить предпробивное напряжение
        mov     _index_b,ax
        mov     ax,259
        xchg    _Phi,ax
        mov     _PhiLast,ax      ; Сохранить угол
        inc     _flp             ; при отсутствии пробоя флаг = 0ffh
        jz      firstSpark       ; на 1-ый пробой
        cmp     _flg,0
        je      noFlv
        cmp     _c100,2          ; повторный пробой
        mov     _c100,0
        mov     _flg,0
        ja      noFlv
        mov     _flv,1           ; Пробой вызван fv
noFlv:  mov     _flo,1           ; Пробой вызван fi/fd
firstSpark:
        cmp     _iLast,ib
        ja       exInt8
        mov     _flb,1           ; Пробой в бестоковую паузу
        jmp     short exInt8

Spark:  cmp     _NumbSpark,2     ; при Стопе устанавливать NumbSpark=10
        jae     nSpark
treatSpark:
        inc     _cnt             ; Обработка пробоев
        jz      pass1
        cmp     _cnt,6           ; через 1 мс после пробоя
        jb      nSpark
        cmp     _flg,2
        je      nSpark
        mov     _uLast,bl        ; последующие проходы при пробое
        cmp     _flg,1
        je      treatDuga
        cmp     bl,60           ; Пока U < 15kv ходить на !! NEW !!!
        ja      uHigh
        mov     _flg,1           ; Общая часть при обработке дугового пробоя
        mov     ax,_Philast
        add     ax,_fv           ; f:=lastF+fv
        cmp     ax,fmin
        jbe     md2
        mov     ax,fmin
        jmp     short md4
md2:    cmp     ax,_fmax
        ja      short md4
        mov     ax,_fmax
md4:    mov     _fvLast,ax
        mov     _cntId,12
        jmp     int8ex
uHigh:  mov     ax,_fi           ; конец обработки искры
        add     ax,_PhiLast
        mov     _Phi,ax
        mov     _fli,2
        mov     _cnt,-1
        jmp     int8ex         ; окончание искры
treatDuga:
        cmp     _flb,1           ; обработка дуги
        je      dugbp
        cmp     _pause,0
        je      dugbp
        mov     al,_pause
        mov     _cpause,al
exitD:  mov     _flg,2             ; Дуга с паузой и окончание дуги и р.бт
        mov     _cnt,-1
        jmp     int8ex

dugbp:  ; Обработка дуги без паузы, и пробоя в безтоковую паузу
        cmp     _iLast,iZero
        ja      int8ex
        dec     _cntId
        jnz     int8ex
        mov     ax,_fvLast
        mov     _Phi,ax
        mov     _flg,2
        mov     _cnt,-1
int8ex: pop     ds
        pop     dx
        pop     bx
        pop     ax
        sti
        iret

mspr:
    mov     dx,378h
    mov     al,2
    out     dx,al
    mov     _b2,1
    mov  al,_flp
    mov     _b5,al
    jmp nspr

InIStAdcU:
        xchg    di,_index
        inc     di
        and     di,sizeArr-1
        mov     dx,158h
        in      ax,dx
        shr     ax,1
        shr     ax,1
        shr     ax,1
        shr     ax,1
        mov     _iLast,al
        mov     _iArr[di],al
        mov     al,_uLast
        mov     _uArr[di],al

; cmp    _flp,0
; jnz  cm0
    cmp    al,150
    ja    mspr
;cm0: cmp    _flp,1
; jnz  cmb1
; cmp    al,120
; ja    mspr
;cmb1: cmp  al,_b1 ; @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
;      ja   mspr
nspr:
        xchg    di,_index
        mov     al,50h          ; StartAdcU
        mov     dx,0158h
        out     dx,al
        mov     bx,_mTail       ; Display
        cmp     bx,_mHead
        je      int8ex
        inc     bx
        and     bx,127
        mov     _mTail,bx
        shl     bx,1
        mov     ax,_messQueue[bx]
        mov     bx,ax
        and     al,0f0h
        or      al,ah
        and     _dtImage,3
        or      al,_dtImage
        mov     dx,_dt          ; 378h
        out     dx,al           ; High 4bit
        shl     bl,1
        shl     bl,1
        mov     bh,al
        or      al,we
        out     dx,al           ; Strobe E - 1
        shl     bl,1
        shl     bl,1
        and     bl,0f0h
        or      bl,ah
        mov     al,bh
        out     dx,al           ; Strobe E - 0
        or      bl,_dtImage
        mov     al,bl
        out     dx,al           ; Lov 4bit
        mov     bh,al
        or      al,we
        nop
        nop
        nop
        out     dx,al           ; Strobe E - 1
        mov     al,bh
        nop
        nop
        nop
        out     dx,al           ; Strobe E - 0
        jmp    short int8ret

ULimit: dec     _cntSec
        jnz     limU
        inc     _iSec
        mov     ax,6347
        mov     _cntSec,ax
        add     _cntMin,ax
        adc     _cntMih,0
limU:   mov     al,_uLast
        cmp     al,_uLim
        jb      chk100
        cmp     _Phi,253
        ja      chk100
    ;    inc     _Phi
chk100: cmp    _flag100,1
        je      int100HgEn
int8ret:pop     ds
        pop     dx
        pop     bx
        pop     ax
        sti
        iret
Int8 ENDP

int100Hg        proc
;????????????????????????????????????????????????????????????????????????
;?          Обработка прерываний по сети ( 100 Hg )                     ?
;????????????????????????????????????????????????????????????????????????
int100HgEn:                     ; Entry point of proc
        mov     _flag100,3      ; исправность ir100
        mov     _numbSpark,0
        push    cx
        push    di
        push    si
        push    bp
        xor     bx,bx           ; KbdScan
        mov     dx,_dt
        mov     ah,0e0h
        mov     ch,_dtImage
        and     ch,2            ; Kt, Phi = "0 Volt"
kbLoop: mov     al,ch
        or      al,ah
        out     dx,al
        inc     dx
        mov     cl,4
        rol     bx,cl
        in      al,dx
        shl     al,1
        and     al,0f0h
        or      bl,al
        dec     dx
        or      ah,8
        shl     ah,1
        jc      kbLoop          ; end Loop Kbd Scan
        cmp     bx,_lastScan
        mov     _lastScan,bx
        jne     endKbdScan
        inc     _cntScan
        cmp     _cntScan,5
        jb      endKbdScan
        mov     cl,4            ; scanCodeStore
        ror     bx,cl
        mov     di,_headKb
        cmp     _queueKb[di],bx
        jne     first
        cmp     bx,-1
        je     endKbdScan
        dec     _cntRepeat
        jnz     endKbdScan
        mov     _cntRepeat,15
        jmp     short repeat
first:  mov     _cntRepeat,40
repeat: xor     ax,ax
        mov     _cntScan,ax
        add     di,2
        and     di,62
        mov     _queueKb[di],bx
        mov     _headKb,di
endKbdScan:     sti
        mov     dx,_pc          ; C2,C3 - Tehn
        in      al,dx           ; C0 - Avar
        mov     ch,al           ; C1 - Dist Rasc => Out
        mov     ah,_fls
jmp stop_q
        test    al,00000001b   ; ? Awar
        jz      wuDu
        or      ah,00000011b   ; wdstttoa  o - do stop
        mov     al,11111101b
        out     dx,al           ; откл. дист. pасцепит.
        or      _fle,80h
wuDu:   test    ah,11000000b   ; ВУ/ДУ
        jz      tech_q
        or      ah,00000010b   ; wdstttoa
        or      _fle,40h
tech_q: and     ah,11100011b    ; погасить биты технол. откл. в Fls
        mov     al,ch
        not     al              ; si il feu
        and     al,00011100b    ; wdstttoa
        jz      stop_q
        or      _fle,al
        or      ah,al
        or      ah,00000010b    ; wdstttoa
stop_q: cmp     bx,0fefbh        ; Command "Stop"  CtrlMinus
        jne     start_q
        and     ah,11111110b    ; погасить бит откл. дист. расц. в Fls
        or      ah,00100010b    ; wdstttoa
;        mov     al,11111111b
;        out     dx,al            ; отмена откл. дист. pасцепит.
        jmp     tir
start_q: cmp    bx,0ffebh        ; Command "Start" CtrlPlus
        jne      tir
        test    ah,00011101b    ; wdstttoa
        jnz     tir
        xor     ah,ah
        mov     al,Fmin
        mov     _Phi,ax
        mov     _Tay,1
ps4:    mov     _ct,1
        mov     _kt,ah          ; коэффициент тpансфоpмации
        mov     _fkt,ah         ; флаг пеpеключения Кт
        and     _flr,0feh       ; kt=0
tir:    mov     _fls,ah         ; Конец Пуск/Стоп
        cmp     ah,0
        je      sWrk
        mov     _Phi,280
        mov     _PhiHpp,280
sWrk:
        cmp     _flp,-1
        je      work
        call    wosst
        jmp     checkHpp
work:
        cmp     _PhiOld,060h    ; пеpеpегулирование пpи пpобоях и
        ja      checkHpp        ; на малых углах не проверять
        mov     ax,_PhiOld      ; PhiOld сохраняется при изменениее Phi
        sar     ax,1
        sar     ax,1            ; f/4 (fmax -> 64) -> по числу замеров тока
        mov     di,_index
        sub     di,60           ; Начало п/п сети + 4 отсчета АЦП (64-4)
        add     di,ax
        and     di,sizeArr-1    ; Момент вкл тиристора + 4 отсчета
        mov     al,_iInf
        sar     al,1
        sar     al,1
        add     al,10
         mov     _b1,al
         mov     ah,_iArr[di]
         mov     _b5,ah
        cmp     _iArr[di],al     ; ? ток > 10+iInf/4
        ja      checkHpp
        ;jb      prr
        mov     al,2
        add     al,_iArr[di]
        inc     di
        and     di,sizeArr-1
        cmp     _iArr[di],al     ; ток растет ?
        ja      checkHpp         ; если да, то нет перерегулирования
prr: ;   inc     _Phi             ; пеpеpегулиpование
        or      _flr,2

checkHpp:
        cmp     _nhp,0
        je      fHigh
        or      _flr,4           ; Чеpезпеpиодное питание
        dec     _chp
        jnz     srmHpp
        cmp     _flf,1
        je      hpMin
        mov     ax,_PhiHpp
        mov     _Phi,ax
        mov     al,_nhf
        mov     _chp,al
        mov     _flf,1
fHigh:
        cmp     _lastSync,0
        jz      Ctrl
        call    srmxmn
    ;    cmp     _flp,-1
    ;    jnz     cfhi
    ;    and      _fle,11111101b
        mov     si,_index_s
        cmp     _Usr[si],25
        ja      cfhi
        dec     si
        and     si,15
        cmp     _Usr[si],25
        ja      cfhi
        cmp     _Isr[si],25
        jb      cfhi
        mov     _Phi,fMin
        mov     _PhiHpp,fMin
        or      _fle,2
cfhi:   call    f_hi
        jmp     short Ctrl

hpMin:  mov     ax,_Phi
    ;    mov     _PhiHpp,ax
        mov     ax,230;fMin
        mov     _Phi,ax
        mov     al,_nhp
        sal     al,1
        mov     _chp,al
        mov     _flf,0
srmHpp:
        cmp     _lastSync,0
        jz      Ctrl
        call    srmxmn
Ctrl:
        mov     al,_Iogr
        add     al,20
        mov     si,_index_s
        cmp     _Isr[si],al
        jbe     Uprob
     ;   and     buf_pc,0efh    ; откл. дист. pасцепит.

Uprob:  mov     al,_Umin[si]     ;Расчет уровня регистрации пробоя
     ;   shr     al,1
        sub     al,20            ; U_isk:=(Umin) - 10kV
        cmp     _flp,-1
        ;je      Up1
        jne      Up2
        ;mov     al,38;14           ; 7 kV
Up1:    mov     _uSpark ,al
Up2:    cmp     _nhp,0          ; pабота на 1-м тиp.
        jne     Ogr_I          ; пpи ЧПП не проверять
l9:     cmp     _Phi,200       ; На малых углах не проверять  ;200
        ja      Ogr_I          ;
c_s:    dec     si
        and     si,0fh
        mov     al,_Umax[si]
        dec     si
        and     si,0fh
        sub     al,_Umax[si]
        cmp     al,20
        jg      tir1
        cmp     al,-20
        jl      tir1
        mov     al,_Isr[si]
        mov     ah,al
        sar     ah,1
        sar     ah,1
        sar     ah,1
        add     ah,8            ; ah <- ( 1/8*I + 8 )
        inc     si
        and     si,0fh
        sub     al,_Isr[si]
        cmp     al,ah
        jg      tir1
        neg     ah
        cmp     al,ah
        jg      Ogr_I
tir1: ;  mov     ax,_Phi          ; pабота на 1-м тиp.
     ;   inc     ax
     ;   or      _fle,20h           ; ??????????????????????
;Ограничение тока и напpяжения (I_действ. по ст.байту)
Ogr_I:  mov     al,_Isr[si]
        mov     ah,_Iogr
        call    Ogr_UI
        ja      o1
        or      _flr,40h
       ; cmp     _t_hand,1
       ; je      o1
        mov     _Tay,4
o1:     mov     al,_Imax[si]
        mov     ah,_ImOgr
        shr     al,1
        call    Ogr_UI
        ja      o2
o2:     mov     al,_Usr[si]
        mov     ah,_Uogr
        call    Ogr_UI
        ja      o3
        or      _flr,80h
o3:     mov     al,_Umax[si]
        mov     ah,_UmOgr
        call    Ogr_UI
        ja      Xx
        or      _flr,80h
Xx:     mov     al,_Uxx
        cmp     _Umax[si],al
        jb      add_T
        mov     al,_Ixx
        cmp     _Isr[si],al
        ja      add_T
        or      _fle,1
        mov     al,_Umax[si]
        mov     ah,_Uxx
        call    Ogr_UI
add_T:
        dec     _cn_c
        jz      loc_T
        jmp     makeRec
loc_T:
        mov     si,_index_s
        mov     _cn_c,16
        lea     bx,_Isr
        call    Sr16
        xor     ah,ah
        add     _Inn,ax
        add     _Iind,ax
        lea     bx,_Usr
        call    Sr16
        xor     ah,ah
        add     _Unn,ax
        add     _Uind,ax
        inc     _dSec                         ; for Indication
        dec     _nMinute
        jnz     s_ind
        cmp     _nIntegr,0
        je      waitRead
        mov     ax,_Unn
        add     _uIntegr,ax
        dec     _nIntegr
waitRead:
      ;  cmp     _t_hand,1
      ;  je      loc4
        call    opt_T          ; ? (if "pause" est change - RET)
loc4:   xor     ax,ax
        xchg    ax,_Inn
        mov     _In_1,ax
        xor     ax,ax
        xchg    ax,_Unn
        mov     _Un_1,ax
        mov     _nMinute,188         ;188*16/50/60 = 1мин

s_ind:
        xor     ax,ax
        xchg    ax,_Iind
        mov     _iInf,al
        xor     ax,ax
        xchg    ax,_Uind
        mov     _uInf,al
makeRec:
        mov     si,_index_s
        cmp     _fkt,0           ; флаг пеpеключ. kt есть?
        jz      commn           ; нет
     ;   call    podsf
commn: ; cmp     _wComm,0
     ;   je      noComm
    ;    call    command
noComm: ;mov     al,timeSys   ; time_sys d h m s
       ; cmp     al,_qsl          ; cek
        jmp     exitdi
         ;фоpмиpование записи для пеpезапуска (1 раз в секунду)
        ;_Phi
        ;Tay
        ;Fv
        ;Fd
        ;RgnTime
        ;wComm
        ;mov     ql,al          ; Запомнить секунды
        ;mov     al,timeSys+1   ; time_sys d h m s
        ;cmp     al,qsm
        ;je      RgnDo
        ;mov     flm,1
        ;mov     qsm,al

exitdi:
        ;dec     timeCount
        ;jz      sysTime
extd:   pop     bp
        pop     si
        pop     di
        pop     cx
        jmp     int8ret     ; ?????????????????????? Return ???????????????????????????
;

;sysTime:
        ;mov     timeCount,100
       ; inc     sysSec
       ; cmp     sysSec,60
       ; jnz     extd

       ; jmp       extd

int100Hg     ENDP

srmxmn  proc    near            ; Сalcule Usr,Isr,Imax,Umax,Umin,Ikv pour 2 p/p
        mov     si,_index
        sub     si,129
        and     si,sizeArr-1    ; Начало массива (128 точек за период )
        mov     _cnSr,2
sr_2:   mov     cx,64           ; Число отсчетов в п/п
        mov     dx,255          ; dh pour Umax, dl pour Umin en period
        sub     bx,bx           ; Обнуление РОН
        mov     di,bx
        mov     bp,bx
       ; mov     _I_kvd,bx
        dec     _cnSr
        jnz     vloop
        ret
vloop:  mov     al,_uArr[si]
        sub     ah,ah
        cmp     al,dh
        ja      msx
ms1:    cmp     al,dl
        jb      msn
ms2:    add     di,ax           ;Usr
        mov     al,_iArr[si]
        cmp     ax,bp
        ja      msix
ms3:    add     bx,ax           ;Isr
       ; call    enable_mul
       ; mul     al
       ; shr     ax,1
       ; shr     ax,1            ; буфеpиpовать и вычислять pеже
       ; shr     ax,1
       ; add     I_kvd,ax        ;I_kv
        inc     si
        and     si,sizeArr-1    ; Начало массива (128 точек за период )
        loop    vloop
        push    si
        mov     si,_index_s
        inc     si
        and     si,0fh
        mov     _index_s,si
        mov     ax,di
        shl     ax,1
        mov     _Usr[si],ah
        shl     bx,1
        shl     bx,1
        mov     _Isr[si],bh
        mov     _Umax[si],dh
        mov     _Umin[si],dl
       ; mov     ax,_I_kvd
       ; mov     _I_kv[si],ax
       ; mov     ax,bp
       ; mov     _Imax[si],al
        pop    si
        jmp     sr_2
msx:    mov     dh,al           ;Umax
        jmp     short ms1
msn:    mov     dl,al           ;Umin
        jmp     short ms2
msix:   mov     bp,ax           ;Imax
        jmp     short ms3
srmxmn  endp

f_hi:
 ;jmp     dcnf
        cmp     _wComm,4 ;osc
        ja      hRet
        cmp     _fls,0
        jnz     hRet
        cmp     _nhp,0
        jz      dcnf
        cmp     _flf,0
        jnz     dcnf
Hret:   ret
dcnf:   dec     _cnFp
        jnz     d_ct
        mov     _cnFp,50
        cmp     _Tay,0
        je      TayOld         ; Tay=0 max 1 sec
        mov     ax,_Phi
        mov     dx,_PhiOld
        mov     bx,dx
        mov     _PhiOld,ax       ; PhiOld <- f
        sub     ax,dx         ; f - PhiOld
        jle     sheckNpr      ; alpha 
        mov     dx,_Phi
        not     dx            ; dl <= -f
        sar     dx,1
        sar     dx,1
        cmp     ax,dx         ; ? (f-PhiOld) < (-f/4)
        jle     modTay
        xor     al,al
        xchg    _Tay,al        ; угол снизился более 1/4
        mov     _oldTay,al
        mov     _ct,1
        mov     ax,255
        sub     ax,bx           ; 255-PhiOld
        sar     ax,1
        sar     ax,1
        sar     ax,1
        add     ax,bx
        cmp     ax,fMin
        jb      mftold
        mov     ax,fMin
mftold: mov     _fTayOld,ax   ; fTayOld = f+(255-f)/8
d_ct:  ; mov     dx,fMin
       ; sub     dx,_Phi
       ; mov     _fvmax,dx
        and     _ct,1ffh
        dec     _ct
        jnz     hRet
pkt:    cmp     _EnablePkt,1
    ;    je      p_kt
        jmp     fhi
TayOld: ; восстановление Tay, если fhi не сделает этого
        mov     al,_oldTay
        cmp     al,4
        ja      lT2
        mov     al,4h
lT2:    mov     _Tay,al
        jmp     short d_ct

modTay:
    ;    cmp     _t_hand,1
    ;    je      d_ct
        sar     dl,1
        sar     dl,1
        sar     dl,1
        inc     dl
    ;    mov     _Tay,dl
        mov     _ct,1
        jmp     d_ct
sheckNpr:
        cmp     _nSpark,0
     ;   mov     _nSpark,0
        jne      d_ct
        cmp     _Tay,8
        jbe     d_ct
     ;   shr     _Tay,1
        jmp      d_ct

p_kt:
        mov     ax,_Phi
        cmp     _kt,1         ; большой коэф.тpансфоpмации?
        je      b_kt         ; да
m_kt:   test    _flr,2        ; малый коэф.тpансфоpмации
        jnz     ktmb
        jmp     fhi           ; f <= fm
ktmb:   mov     _fm,ax
        add     ax,_fkm
     ;   mov     _Phi,ax
        mov     al,_Usr[si]
        mov     _Up,al
        mov     al,_Isr[si]
        mov     _Ip,al
        mov     _kt,1
        or      _flr,1
    ;    and     buf_pc,11110111b
        mov     _fkt,1
        mov     _ct,4
        ret
b_kt:
        cmp     _fkt,8        ; большой коэф.тpансфоpмации
        jne     mbk
        mov     al,_Usr[si]
        add     al,10
        cmp     al,_Up
        ja      mbk
        mov     _EnablePkt,0
        jmp     short mmk
mbk:    cmp     ax,_fg        ; большой Кт
        jb      fhi          ; если f >= fg
        add     ax,_fkb       ; пеpеход с большого Кт на малый
      ;  mov     _Phi,ax
        mov     al,_Usr[si]
        mov     _Up,al
        mov     al,_Isr[si]
        mov     _Ip,al
        mov     _fkt,1
mmk:    mov     _kt,0
        and     _flr,0feh
   ;     or      buf_pc,00001000b
        mov     _ct,4
        ret
fhi:
        mov     si,_index_s
        cmp     _Usr[si],10
        ja      f_maxm
        cmp     _Phi,fMin
        ja      f_maxm
        cmp     _index_s,8
        jne     f_maxm
f2:     mov     _ct,2
        mov     ax,fmin
        jmp     short mvf
f_maxm: mov     _flk,0
        cmp     _Usr[si],60
        mov     dx,_fmax
        jbe     modf
        mov     al,-60
        add     al,_Usr[si]
        shl     al,1
        xor     ah,ah
        add     dx,ax
modf:   mov     al,_Tay
        xor     ah,ah
        shl     ax,1
        shl     ax,1
        cmp     ax,0
        jne     l8
        sub     _Phi,7          ;Ускоренный подъем после пакета пробоев
        or      ax,1
l8:     mov     _ct,ax
        mov     ax,_Phi
        dec     ax
        cmp     ax,dx           ; fmax - рабочее значение
        ja      mvf
        mov     ax,dx
mvf:    mov     _Phi,ax
        mov     _PhiHpp,ax
        cmp     _Tay,0
        jne     locF
        cmp     ax,_fTayOld
        ja      locF
        mov     al,_oldTay
      ;  cmp     _t_hand,1
      ;  je      lT1
        cmp     al,5
        ja      lT1
        mov     al,5
lT1:    mov     _Tay,al
locF:     ret

ok:     ; Проверка отсутствия обратной короны
        dec     _cnHi
        jz      detect
        js      ini4
        ret
detect:
        mov     si,_index_s
        mov     dl,_Umin[si]
        add     dl,2
        mov     dh,_Isr[si]
        sub     dh,6
        jns     ok1
        xor     dh,dh
ok1:    cmp     dl,_Umn1         ; Umin - Umn1
        ja      exNoOk
        cmp     dh,_Isr1         ; Usr - Usr1
        jbe     exNoOk
        or      _flr,8
        cmp     _enableHpp,0
        je      okF
        mov     _cnHppOk,200      ; ~ на 1 час
        mov     _nhp,2            ; вкл. ЧПП
      ;  cmp     _t_hand,1
      ;  je      lk1
        mov     _Tay,32
        mov     _ct,64
lk1:    ret
ini4:   mov     _cnHi,4
        mov     si,_index_s
        mov     dl,_Umin[si]
        mov     dh,_Isr[si]
        mov     _Umn1,dl
        mov     _Isr1,dh
        ret
exNoOk: ret
okF:  ;  mov     al,f            ; огpаничение ОК углом
       ; mov     fmax,al
      ;  add     al,20
      ;  jc      ffm
      ;  cmp     al,fmin
      ;  jbe     outff
ffm:  ;  mov     al,fmin
outff:;  mov     f,al
      ;  out     tf,al
        ret

;ok_h:   add     ah,2
;        cmp     ah,v_f
;        jb      hp_
;        inc     nhp
;        cmp     nhp,20
;        jbe     ex_ok
;        mov     nhp,0
;        mov     enableHhpp,0    ; безуспешное пpоpеживание
;        jmp

;hp_:    cmp     nhp,1
;        je      ex_ok
;        dec     nhp
;        jmp     ex_ok
;
;
;end_ok: dec     cn_ok
;        jnz     ex_ok
;        mov     fla,0
;        mov     fmax,fmx
;

;        mov     al,Umax[si]     ; вычисление скоpости pазpяда
;        sub     al,2            ; если Umax-Usr < 2 то не вычисляем
; вычисление скоpости pазpяда емкости эл. фильтpа
;        mov     si,index
;        neg     si
;        and     si,40h          ; Начало массива мгн. зн.
;mnk:    inc     si
;        cmp     uArr[si],al    ; Поиск максимума U
;        jb      mnk
;mnkx:   inc     si
;        cmp     uArr[si],al
;        jae     mnkx
;        mov     ah,uArr[si]    ; ah < - максимум
;        add     si,8
;        sub     ah,uArr[si]    ; v
;        cmp     nhp,0
;        jne     ok_h
;        mov     v_f,ah          ; буфеpиpуем скоpость pазpяда
        ret
podsf   proc    near
; подстpойка угла после пеpеключения
; коэффициента тpансфоpмации
        cmp     _fls,0
        jne      a
        inc     _fkt
        cmp     _fkt,8
        jbe     pret
        cmp     _fkt,220
        ja      a
     ;   mov     _zm1_4[5],057h
        mov     ah,_Up
        sub     ah,_Usr[si]     ; up - u < 2
        jle     okon_p          ; нет
        and     ah,1fh
      ;  mov     al,f
        sub     al,ah           ; f = f - du/2
       ; cmp     al,fmax+50
        jbe     okon_p          ; нет
ftt:   ; out     tf,al
       ; mov     f,al
pret:   ret
okon_p:; mov     al,f
        cmp     _kt,1
        je      b_fk
        ;sub     al,fg      ; Было пеpекл.  Б - М
        add     al,5       ; Вычисление fkb
        cmp     al,0f0h
        ja      fkbmin
        cmp     al,0a0h
        jb      fkbmax
kb:   ;  mov     fkb,al
        jmp     short a
b_fk:  ; sub     al,fm
        add     al,5       ;Вычисление fkm
        cmp     al,20
        jb      fkmmin
        cmp     al,150
        ja      fkmmax
km:    ; mov     fkm,al
a:      mov     _fkt,0
        ret
fkbmin: mov     al,0f0h
        jmp     short kb
fkbmax: mov     al,0a0h
        jmp     short kb
fkmmin: mov     al,40
        jmp     short km
fkmmax: mov     al,100
        jmp     short km
podsf   endp

Ogr_UI: cmp     ah,al
        ja      ogret
        sub     al,ah           ; Isr-Iogr
        xor     ah,ah
        add     ax,_Phi          ; f:=f + delta_I
        jc      mo1
        cmp     ax,fmin
        jbe     mo2
mo1:    mov     ax,fmin
mo2:    cmp     _nhp,0
        jne     ogHpp
        mov     _Phi,ax
ogret:  ret
ogHpp:;  mov     _PhiHpp,ax
        ret

Opt_T   proc        ;Оптимизация Tay
;Сpеднее напpяжение на электpофильтpе pастет?
        mov     ax,_Unn
        cmp     ax,_Un_1
        jb      cm_Up           ; нет
;Pазность между током на N-шаге и N-1 шаге
;меньше допустимого значения?
        mov     ax,_Inn
        sub     ax,20
        cmp     ax,_In_1         ; (Inn-Idop) - In_1
        jg      inv_T           ; Нет
m4:     cmp     _nSpark,0          ; Да. Пpобои есть?
        jz      cm_df           ; Нет пpобоев
;Pасчет скоpости подъема напpяжения
T_rc:   cmp     _fln,1
        je      mul_T
        cmp     _Tay,16
        jbe     mt1
        mov     cl,2
        shr     _Tay,cl
        ret
mt1:    mov     _Tay,4
        ret
mul_T:  cmp     _Tay,10h
        jae     mt80
        mov     cl,1
        shl     _Tay,cl
        ret
mt80:   mov     _Tay,20h
        ret
;Пpобивная пpочность уменьшается?
cm_Up:
        cmp     _nSpark,0        ; Пpобои есть?
        jz      mfn0            ; Нет пpобоев
        mov     di,_index_p
        mov     cx,_nSpark
        cmp     cx,1fh
        jbe     srp1
        mov     cx,1fh
srp1:   mov     bx,cx
        mov     di,_index_p
        mov     ax,0
        mov     dx,ax
lsr:    mov     dl,_uSpr[di]
        add     ax,dx
        dec     di
        and     di,1fh
        loop    lsr
        div     bl
        mov     ah,_Uprn         ; Сpеднее пpобивное напp.
        mov     _Uprn,al         ; сумма uSpr n
        mov     _nSpark,0
        cmp     al,ah           ; Uprn-Uprn_1
        jae     inv_T
        mov     _fln,1
        jmp     T_rc
;Инвеpcия напpавления изменения  Tay (fln)
inv_T:  xor     _fln,1
        jmp     T_rc
cm_df: ; mov     al,fn_1         ; нет пpобоев
       ; mov     ah,f
       ; mov     fn_1,ah         ; fn_1 <- f
        sub     al,ah           ; f - fn_1
       ; cmp     ax,fd           ; f уменьшился большe,чем отpаботка?
        ja      mfn0            ; да,необходимо уменьшить Tay
        ret                     ; нет, Tay не изменять
mfn0:   mov     _fln,0
        jmp     T_rc
Opt_T  endp

Sr16:   xor     ax,ax           ; added value in [bx]
        mov     dx,ax
        mov     cx,16
sr_16:  mov     dl,[bx][si]
        dec     si
        and     si,15
        add     ax,dx
        loop    sr_16
        mov     cl,4
        sar     ax,cl
        ret                     ;return Sr to <al>

hppWosst:
        mov     ax,_fd
        inc     ax
        add     _PhiHpp,ax
        xor     al,al
        mov     _fli,al
        mov     _flg,al
        dec     al
        mov     _flp,al
        ret
Wosst   proc          ; Восстановление напpяжения, ортимизация fi/fd/fv
        cmp     _nhp,0
        jnz      hppWosst
        cmp     _flg,2
        jne     cfli
        jmp     ofv             ; на регулирование fv/fd
cfli:   cmp     _flg,0
        jnz     locret
        cmp     _fli,2          ; на регулирование fi/fb
        jne     locret
ofi:    or      _flr,10h
        inc     _c100             ; регулирование fi
        cmp     _c100,10
        jne     locret
        mov     _c100,0
        cmp     _flo,1
        jne     prob1
        mov     al,_flt          ; тип пробоя: 0-i; 2-d/b
        xor     ah,ah
        mov     _flt,ah          ; если искpа flt := 0
        mov     _flo,ah          ; reset flo
        mov     di,ax
        jmp     word ptr cs:add_f[di]
prob1:  xor     al,al           ; Первый искровой пробой
        mov     _fli,al
        dec     al
        mov     _flp,al
        cmp     _coi,0
        jz      sub_fi
        dec     _coi
locRet: ret

add_f:  dw      offset add_fi
        dw      offset add_fd
add_fi: ; вторичный пробой, обусловленный малым fi
        mov     ax,_fi
        add     ax,2
        xor     ah,ah
        mov     dx,_PhiLast
        not     dx
        mov     cl,4
        shr     dx,cl           ; fimax = f/16
        add     dx,2
        cmp     ax,dx
        jb      loadfi
        mov     ax,dx
loadfi: mov     _fi,ax
        mov     _coi,8h
        ret

sub_fi: mov     ax,_fi       ; уменьшение fi
        sub     ax,1
        jc      ldfim
        cmp     ax,fimin
        jae     ldfi
ldfim:  mov     ax,fimin
ldfi:   mov     _fi,ax
        mov     _coi,4
lc1ret: ret

dcntp:  dec     _cpause
        jnz     lc1ret
        mov     ax,_Philast
        add     ax,_fv           ; f:=lastF+fv
        cmp     ax,fmin
        jbe     mv2
        mov     ax,fmin
        jmp     short mv4
mv2:    cmp     ax,_fmax
        ja      mv4
        mov     ax,_fmax
mv4:    mov     _Phi,ax
        ret

ofv:
        cmp     _cpause,0
        jne     dcntp
        inc     _c100             ; Ортимизация fv/fd
        cmp     _c100,2
        je      outfd
        cmp     _c100,3
        jne     eOfSpark
        jmp     c_flv           ; на регулирование fv

eOfSpark:                       ; Окончание обработки дугового пробоя
        cmp     _c100,12
        jne     lc1ret
        xor     al,al
        mov     _c100,al
        mov     _flb,al
        mov     _flg,al
        mov     _fli,al
        mov     _flp,-1
        cmp     _flo,al
        jz      dcoi
        mov     al,_flt          ; 0-i, 2-d
        mov     _flt,2           ; дуга, flt:=2
        xor     ah,ah            ; Повторный пробой,
        mov     _flo,ah          ; увеличить отработку
        mov     di,ax            ; для соотв. типа
        jmp     word ptr cs:add_f[di]

outfd:          ; Вывод f+fd через 1 период после подачи f+fv
        or      _flr,20h
        mov     ax,_fd           ; f:=fbuf+fd
        mov     dx,_PhiLast
        not     dx
        mul     dx
        mov     ax,dx
        cmp     ax,0
        jnz     loclp
        inc     ax
loclp:  mov     cl,_flp
        inc     cl
        test    cl,cl
        jz      lfd
        cmp     cl,4
        jb      shrCl
        mov     cl,4
shrCl:  shr     ax,cl           ; уменьшить отработку для повторных пробоев
        or      ax,1
lfd:    add     ax,_PhiLast
        cmp     ax,fmin
        jbe     storeF
        mov     ax,fmin
storeF: mov     _Phi,ax
        ret
dcoi:           ; Уменьшение fd; увеличения при повт. пpобоях
   ;     cmp     _t_hand,0        ; проверить флаг разрешения режима подбора Тау
    ;    jne     lcod
        mov     dx,_fd           ; установка Тау по FD,
        mov     ax,100; _periodSpark  ; по заданному периоду пробоев
        div     dl
mTay:   mov     _Tay,al
lcod:   cmp     _cod,0
        jz      mvfi
        dec     _cod
        ret
mvfi:   mov     ax,_fd
        mov     bx,ax
        mov     cl,4
        sar     bx,cl
        inc     bx
        sub     ax,bx
        jc      sdfdm
        cmp     ax,fdmin
        jae     sdfi
sdfdm:  mov     ax,fdmin
sdfi:   mov     _fd,ax
        mov     _cod,2
        ret
c_flv:  cmp     _nhp,0           ; регулирование fv
        jne     fv_hpp
        cmp     _flv,0
        je      fld0
        mov     di,_index_p      ; 2_ой пробой пpи восстановлении
        mov     dl,_uSpr[di]     ; uSpr[n]
        dec     di
        and     di,01fh
        mov     al,_uSpr[di]     ; uSpr[n-1]
        shr     al,1
        cmp     dl,al           ; Upr-(Upr[n-1]/2)
        jae     mvp
        mov     byte ptr _cpaus,4
        add     _pause,1         ; si uSpr tombe
        cmp     _pause,5
        jbe     mvp
        mov     _pause,2
        add     _fv,4
mvp:    mov     ax,_fv
        add     ax,5
        cmp     ax,_fvMax         ; fvMax = fMin - f
        jle     tfv              ; Уменьшение угла при форсировке не более, чем до Fmin
fv_hpp: mov     ax,_fvmax
tfv:    mov     _fv,ax
        mov     _flv,0
        mov     _cov,4
        ret
fld0:                              ; первая дуга
        cmp     _pause,0
        jz      modfv
        dec     byte ptr _cpaus   ; первая дуга
        jnz     modfv
        mov     byte ptr _cpaus,2
        dec     _pause
        jns     modfv
        mov     _pause,0
modfv:  mov     di,_index
        sub     di,64
        mov     cx,20h
        xor     ax,ax
lmx:    dec     di              ; найти Мах U пpи фоpсиpовке
        and     di,sizeArr-1
        cmp     al,_uArr[di]
        jae     lomx
        mov     al,_uArr[di]
lomx:   loop    lmx
        add     al,4            ; Umax + 2kV
UBp:    mov     si,_index_s
        sub     si,8
        and     si,15
        mov     dl,_Umax[si]
        cmp     al,dl           ; UmaxWosst - Umax
        jb      ccov
        mov     ax,_fv
        add     ax,02           ; Увеличение fv
        cmp     ax,_fvmax        ; fvmax = Fmin - Phi
        jle     ltv
        mov     _cov,4
        mov     ax,_fvmax
ltv:    mov     _fv,ax
        ret
ccov:   cmp     _cov,1           ; Уменьшение fv
        jle     fvsub
        dec     _cov
        ret
fvsub:  mov     ax,_fv
sdx:    sub     ax,1
        mov     _cov,4
        cmp     ax,fvmin
        jge     ltv
       ; mov     ax,fvmin
        jmp     short ltv

add_fd:         ; вторичный пробой, обусловленный малым fd
        mov     ax,_fd
        mov     dx,ax
        sar     dx,1
        sar     dx,1
        sar     dx,1
        add     ax,dx
        add     ax,4
        jc      loaddm
        cmp     ax,fdmax
        jb      loadfd
loaddm: mov     ax,fdmax
loadfd: mov     _fd,ax
        mov     _cod,8
exfd:   ret

Wosst  endp

adcIn   proc
         mov dx,379h
         in  al,dx
         mov ah,al
         mov al,0ffh
         inc dx
         out dx,al
         shl ah,1
         dec dx
         in  al,dx
         shr al,1
         shr al,1
         shr al,1
         and ax,0f00fh
         or  al,ah
         ret
adcIn   endp

StAdcI   proc
        mov dx,37ah             ; Start ADC i
        mov al,0fch
        out dx,al
        jmp li01
li01:   mov al,0feh
        out dx,al
         ret
StAdcI   endp

include rn.asm

RGI_TEXT ends

END
