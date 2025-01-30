// Gestion des onglets
document.querySelectorAll(".tab-link").forEach((tabLink) => {
    tabLink.addEventListener("click", function (e) {
        e.preventDefault();

        // Supprimer la classe "active" de tous les onglets
        document.querySelectorAll(".tab-link").forEach((link) => link.classList.remove("active"));
        document.querySelectorAll(".content").forEach((content) => content.classList.remove("active"));

        // Activer l'onglet cliqué
        this.classList.add("active");
        const targetTab = document.getElementById(this.getAttribute("data-tab"));
        targetTab.classList.add("active");
    });
});
