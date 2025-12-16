#!/bin/bash

# ----------------------------------------------------
# 1. Définir les branches et les chemins
# ----------------------------------------------------
BASE_BRANCH="origin/develop"
CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD)

# >>> DÉFINITION DE DEUX RÉPERTOIRES DE SORTIE CLAIRS <<<

# Répertoire pour les fichiers de DIFFERENCES (Patchs)
PATCH_OUTPUT_DIR="patchs/diffs_${CURRENT_BRANCH}_vs_develop"

# Répertoire pour les fichiers au CONTENU COMPLET (Brut)
BRUT_OUTPUT_DIR="patchs/complets_${CURRENT_BRANCH}_bruts"

# Fichiers spécifiques DANS les sous-modules (seront traités en BRUT)
SUBMODULE_TARGETS=(
    "protobufs:meshtastic/telemetry.proto"
)

# Filtre pour les fichiers du SUPER-PROJET (seront traités en PATCHS)
SUPER_PROJECT_FILTER="platformio.ini src/ "
SUPER_PROJECT_FILTER="platformio.ini src/ meshTestic/ protobufs/"

echo "=========================================================="
echo "Préparation pour la branche : [$CURRENT_BRANCH]"
echo "Patchs (Différences) dans : $PATCH_OUTPUT_DIR/"
echo "Fichiers Bruts (Complets) dans : $BRUT_OUTPUT_DIR/"
echo "=========================================================="

# Créer les deux répertoires et obtenir leurs chemins absolus
mkdir -p "$PATCH_OUTPUT_DIR" "$BRUT_OUTPUT_DIR"
FULL_BRUT_OUTPUT_DIR="$(pwd)/$BRUT_OUTPUT_DIR"


# ----------------------------------------------------
# 2. LOGIQUE POUR LES FICHIERS BRUTS (SOUS-MODULES)
# ----------------------------------------------------

for TARGET in "${SUBMODULE_TARGETS[@]}"; do
    SUBMODULE_NAME=$(echo "$TARGET" | cut -d ':' -f 1)
    INTERNAL_FILE_PATH=$(echo "$TARGET" | cut -d ':' -f 2)
    FULL_PATH="$SUBMODULE_NAME/$INTERNAL_FILE_PATH"

    echo "--- Traitement Brut (Fichier Complet) : $FULL_PATH ---"

    CURRENT_SUBMODULE_SHA=$(git ls-tree $CURRENT_BRANCH -- "$SUBMODULE_NAME" | awk '{print $3}')

    if [ -z "$CURRENT_SUBMODULE_SHA" ]; then
        echo "  Erreur: Impossible d'obtenir le SHA du sous-module $SUBMODULE_NAME." >&2
        continue
    fi

    if [ -d "$SUBMODULE_NAME" ]; then
        FILE_NAME_SAFE=$(echo $FULL_PATH | tr / _)
        OUTPUT_FILE="$FULL_BRUT_OUTPUT_DIR/$FILE_NAME_SAFE"
        
        # Exécution de GIT SHOW DANS le sous-module
        (
            cd "$SUBMODULE_NAME" && \
            git show $CURRENT_SUBMODULE_SHA:"$INTERNAL_FILE_PATH" > "$OUTPUT_FILE"
        )
        
        if [ -s "$OUTPUT_FILE" ]; then
            echo "  ✅ Fichier complet généré : $OUTPUT_FILE"
        else
            rm -f "$OUTPUT_FILE"
            echo "  ⚠️ Le fichier est vide ou n'existe pas dans ce commit. Fichier ignoré."
        fi
    else
        echo "  Erreur: Le répertoire du sous-module $SUBMODULE_NAME n'existe pas localement." >&2
    fi
done

# ----------------------------------------------------
# 3. LOGIQUE POUR LES PATCHS (SUPER-PROJET)
# ----------------------------------------------------

echo "--- Traitement Patch (Différences) des fichiers du Super-Projet ---"

FULL_PATCH_OUTPUT_DIR="$(pwd)/$PATCH_OUTPUT_DIR"

# On filtre les fichiers du super-projet qui ont été modifiés
FILES=$(git diff --name-only $BASE_BRANCH $CURRENT_BRANCH -- $SUPER_PROJECT_FILTER)

for FILE in $FILES; do
    PATCH_NAME=$(echo "$FILE" | tr / _)
    OUTPUT_FILE="$FULL_PATCH_OUTPUT_DIR/$PATCH_NAME.patch" # Notez l'extension .patch ajoutée

    echo "  -> Génération de patch pour : $FILE"

    # Utiliser git diff pour générer le patch (différence)
    git diff $BASE_BRANCH $CURRENT_BRANCH -- "$FILE" > "$OUTPUT_FILE"

    if [ -s "$OUTPUT_FILE" ]; then
        echo "  ✅ Patch (diff) généré : $OUTPUT_FILE"
    else
        # Si le fichier est vide, c'est que le diff est vide ou que le fichier n'a pas été modifié.
        rm -f "$OUTPUT_FILE"
        echo "  ⚠️ Le fichier $FILE n'a pas de différence significative. Patch ignoré."
    fi
done

echo "=========================================================="
echo "Opération terminée."
echo "Les patchs sont dans : $PATCH_OUTPUT_DIR/"
echo "Les fichiers bruts sont dans : $BRUT_OUTPUT_DIR/"
