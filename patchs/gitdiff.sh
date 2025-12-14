#!/bin/bash

# ----------------------------------------------------
# 1. Définir les branches et les chemins
# ----------------------------------------------------
BASE_BRANCH="origin/develop"

# La branche actuelle (la source des modifications)
CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD)

# Les chemins ciblés (incluant les sous-modules)
# NOTE: Assurez-vous que les chemins (meshtestic/ et protobufs/) sont corrects
CHEMINS_CIBLES="platformio.ini src/ meshtestic/ protobufs/"

# Nom du répertoire de sortie
OUTPUT_DIR="patchs/individual_patches_from_${CURRENT_BRANCH}"

# ----------------------------------------------------
# 2. VÉRIFICATION DE SÉCURITÉ
# ----------------------------------------------------
if [ "$CURRENT_BRANCH" = "HEAD" ] || [ "$CURRENT_BRANCH" = "" ]; then
    echo "ERREUR : HEAD est détaché (detached HEAD) ou la branche n'a pu être déterminée." >&2
    echo "Veuillez vous assurer d'être sur une branche nommée." >&2
    exit 1
fi

echo "=========================================================="
echo "Analyse de la branche en cours : [$CURRENT_BRANCH]"
echo "Comparaison : [$BASE_BRANCH] vs [$CURRENT_BRANCH] (avec sous-modules)"
echo "Filtres : $CHEMINS_CIBLES"
echo "Les patchs seront stockés dans : $OUTPUT_DIR/"
echo "=========================================================="

# ----------------------------------------------------
# 3. Préparation
# ----------------------------------------------------
mkdir -p "$OUTPUT_DIR"

# ----------------------------------------------------
# 4. Récupérer les PATHS des fichiers affectés, y compris les sous-modules
# ----------------------------------------------------
# On utilise un pipe et `awk` pour extraire uniquement les noms de fichiers
# à partir de la sortie brute de 'git diff', ce qui fonctionne avec les sous-modules.
# On utilise l'option --no-ext-diff pour éviter les problèmes avec les outils externes
# et --submodule=log pour forcer le diff dans le sous-module.
echo "Extraction des fichiers modifiés..."

FILES=$(git diff --name-only --submodule=log $BASE_BRANCH $CURRENT_BRANCH -- $CHEMINS_CIBLES)

if [ -z "$FILES" ]; then
    echo "Aucun fichier modifié trouvé dans le périmètre ($CHEMINS_CIBLES) entre $BASE_BRANCH et $CURRENT_BRANCH."
    exit 0
fi

# ----------------------------------------------------
# 5. Boucler sur chaque fichier et créer son patch individuel
# ----------------------------------------------------
for FILE in $FILES; do
    # Création d'un nom de fichier sûr pour le patch (remplace les '/' par des '_')
    PATCH_NAME=$(echo "$FILE" | tr -c '[:alnum:].' '_')
    
    echo " -> Génération du patch pour : $FILE"
    
    # Exécuter git diff pour le fichier spécifique. 
    # NOTE : Pour les sous-modules, Git ne génère pas de patch fichier par fichier. 
    # Il génère un entête de diff du sous-module.
    # Pour un fichier classique, le diff sera complet.
    git diff $BASE_BRANCH $CURRENT_BRANCH -- "$FILE" > "$OUTPUT_DIR/$PATCH_NAME.patch"
done

echo "----------------------------------------------------------------------"
echo "Opération terminée. Les patchs individuels se trouvent dans le répertoire $OUTPUT_DIR/."