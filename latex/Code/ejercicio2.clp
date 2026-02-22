; ############################################
; ############## PLANTILLAS ##################
; ############################################
; Plantilla para describir un ingrediente
(deftemplate ingrediente
   (slot tipo) ; fruta, carne...
   (slot nombre) ; manzana, ternera...
   (slot calorias)) ; 40, 220...

; Plantilla para describir un menu
(deftemplate menu
   (slot primero)
   (slot segundo)
   (slot postre)
   (slot calorias-totales))

; Plantilla para describir una restriccion calorica
(deftemplate restriccion
   (slot max-calorias))
; ############################################
; ############################################


; ############################################
; ############## HECHOS ##################
; ############################################
; Definir hechos iniciales
(deffacts ingredientes-iniciales
   (ingrediente (tipo fruta) (nombre manzana) (calorias 40))
   (ingrediente (tipo carne) (nombre ternera) (calorias 220))
   (ingrediente (tipo pescado) (nombre salmon) (calorias 180))
   (ingrediente (tipo vegano) (nombre tofu) (calorias 120))
   (ingrediente (tipo pastel) (nombre milhojas) (calorias 300))
   (ingrediente (tipo cafe) (nombre cafe) (calorias 5)))
; ############################################
; ############################################


; ############################################
; ############### FUNCIONES ##################
; ############################################
; Funcion para imprimir el informe de menus generados
(deffunction imprimir-informe-menus ()
   (printout t crlf "########## MENUS GENERADOS ##########" crlf)

   (bind ?menus (find-all-facts ((?m menu)) TRUE))

   (if (eq (length$ ?menus) 0) then
      (printout t "No se ha generado ningun menu." crlf)
   else
      (foreach ?m ?menus
         (printout t crlf "=== MENU ===" crlf
                        "  Primero: " (fact-slot-value ?m primero) crlf
                        "  Segundo: " (fact-slot-value ?m segundo) crlf
                        "  Postre: " (fact-slot-value ?m postre) crlf
                        "  Calorias totales: " (fact-slot-value ?m calorias-totales) crlf
                        "--------------------------------------" crlf))
   )

   (printout t "######## FIN DEL INFORME ########" crlf)
)

; Funcion para ver restriccion calorica
(deffunction ver-restriccion ()
   (bind ?r-list (find-fact ((?f restriccion)) TRUE))

   (if (eq ?r-list FALSE)
      then
         (printout t "No hay ninguna restricción calórica establecida." crlf)
      else
         (bind ?r (nth$ 1 ?r-list))
         (printout t "Restricción calórica actual: "
                     (fact-slot-value ?r max-calorias)
                     " kcal" crlf)
   )
)


; Funcion de menu interactivo
(deffunction menu ()
   (printout t crlf "=== MENU DIETETICO ===" crlf
                "0. Inicializar sistema (reset)" crlf
                "1. Añadir ingrediente" crlf
                "2. Eliminar ingrediente" crlf
                "3. Ver ingredientes" crlf
                "4. Establecer restriccion calorica" crlf
                "5. Ver restriccion calorica actual" crlf
                "6. Generar menus" crlf
                "7. Ver menus generados" crlf
                "8. Guardar hechos en fichero" crlf
                "9. Cargar hechos desde fichero" crlf
                "10. Salir" crlf
                "Opcion: ")

   (bind ?op (read))

   ;; 0 - RESET
   (if (eq ?op 0) then
      (reset)
      (assert (restriccion (max-calorias -1)))
      (printout t "Sistema inicializado con ingredientes por defecto." crlf)
   )

   ;; 1 - AÑADIR INGREDIENTE
   (if (eq ?op 1) then
      (printout t "Introduce ingrediente en formato: nombre tipo calorias" crlf)
      (bind ?linea (readline))
      (bind ?tok (explode$ ?linea))
      (bind ?n (nth$ 1 ?tok))
      (bind ?t (nth$ 2 ?tok))
      (bind ?c (nth$ 3 ?tok))
      (assert (ingrediente (nombre ?n) (tipo ?t) (calorias ?c)))
      (printout t "Ingrediente añadido correctamente." crlf)
   )

   ;; 2 - ELIMINAR INGREDIENTE
   (if (eq ?op 2) then
      (facts)
      (printout t "Introduce ID del hecho a eliminar: ")
      (bind ?id (read))
      (retract ?id)
      (printout t "Hecho eliminado." crlf)
   )

   ;; 3 - VER INGREDIENTES
   (if (eq ?op 3) then
      (printout t crlf "=== INGREDIENTES DISPONIBLES ===" crlf)
      (foreach ?f (find-all-facts ((?x ingrediente)) TRUE)
         (printout t "[id " (fact-index ?f) "] "
                     (fact-slot-value ?f nombre) " | "
                     (fact-slot-value ?f tipo) " | "
                     (fact-slot-value ?f calorias) " kcal" crlf))
   )

   ;; 4 - ESTABLECER RESTRICCIÓN CALÓRICA
   (if (eq ?op 4) then
      (printout t "Introduce el máximo de calorías permitidas: ")
      (bind ?max (read))

      ;; eliminar restricciones anteriores
      (foreach ?r (find-all-facts ((?x restriccion)) TRUE)
         (retract (fact-index ?r)))

      ;; añadir nueva restricción
      (assert (restriccion (max-calorias ?max)))
      (printout t "Restricción establecida." crlf)
   )

   ;; 5 - VER RESTRICCIÓN CALÓRICA
   (if (eq ?op 5) then
      (ver-restriccion)
   )

   ;; 6 - GENERAR MENÚS
   (if (eq ?op 6) then
      (run)
      (printout t "Generación de menús completada." crlf)
   )

   ;; 7 - VER MENÚS
   (if (eq ?op 7) then
      (printout t crlf "=== MENÚS GENERADOS ===" crlf)
      (bind ?menus (find-all-facts ((?m menu)) TRUE))
      (if (eq (length$ ?menus) 0) then
         (printout t "No se han generado menús." crlf)
      else
         (foreach ?m ?menus
            (printout t "Menú:" crlf
                        "  Primero: " (fact-slot-value ?m primero) crlf
                        "  Segundo: " (fact-slot-value ?m segundo) crlf
                        "  Postre: " (fact-slot-value ?m postre) crlf
                        "  Calorías totales: " (fact-slot-value ?m calorias-totales) crlf crlf)))
   )

   ;; 8 - GUARDAR HECHOS
   (if (eq ?op 8) then
      (printout t "Nombre del fichero: ")
      (bind ?fname (readline))
      (if (eq ?fname "") then (bind ?fname "menus.clp"))
      (save-facts ?fname local)
      (printout t "Hechos guardados en " ?fname crlf)
   )

   ;; 9 - CARGAR HECHOS
   (if (eq ?op 9) then
      (printout t "Nombre del fichero a cargar: ")
      (bind ?fname (readline))
      (if (neq ?fname "") then
         (load-facts ?fname)
         (printout t "Hechos cargados desde " ?fname crlf)
      else
         (printout t "No se especificó fichero." crlf))
   )

   ;; 10 - SALIR
   (if (eq ?op 10) then
      (printout t "Saliendo del sistema..." crlf)
      (halt)
   )

   ;; VOLVER AL MENÚ
   (if (neq ?op 10) then
      (menu))
)
; ############################################
; ############################################


; ############################################
; ################ REGLAS ####################
; ############################################
; Regla para crear un menu
(defrule crear-menu
   (restriccion (max-calorias ?max))

   ;; PRIMER PLATO: no puede ser fruta, pastel ni café
   (ingrediente (nombre ?p1)
                (tipo ?t1&~fruta&~pastel&~cafe)
                (calorias ?c1))

   ;; SEGUNDO PLATO:
   ;; - debe ser carne, pescado o vegano
   ;; - NO puede tener el mismo tipo que el primer plato
   (ingrediente (nombre ?p2)
                (tipo ?t2&~?t1&:(or (eq ?t2 carne)
                                    (eq ?t2 pescado)
                                    (eq ?t2 vegano)))
                (calorias ?c2))

   ;; POSTRE:
   ;; - fruta, pastel o café
   ;; - NO puede tener el mismo tipo que los otros dos
   (ingrediente (nombre ?p3)
                (tipo ?t3&~?t1&~?t2&:(or (eq ?t3 fruta)
                                         (eq ?t3 pastel)
                                         (eq ?t3 cafe)))
                (calorias ?c3))

   ;; Restricción calórica
   (test (<= (+ ?c1 ?c2 ?c3) ?max))

   =>
   (assert (menu
      (primero ?p1)
      (segundo ?p2)
      (postre ?p3)
      (calorias-totales (+ ?c1 ?c2 ?c3))))
)
;; & (AND), ~ (distinto), ?t1 (variable que captura el valor del slot tipo)
;; el postre tiene que ser fruta, pastel o cafe
	;; (tipo ?t3&:(or (eq ?t3 fruta) (eq ?t3 pastel) (eq ?t3 cafe)))
;; (+ ?c1 ?c2 ?c3) --> la suma de calorias

; Regla "sin-solucion", en este caso "no-comas"
(defrule no-comas
   (restriccion (max-calorias ?max))
   (not (menu (calorias-totales ?c&:(<= ?c ?max))))
   =>
   (printout t "No comas." crlf)
)
; ############################################
; ############################################
