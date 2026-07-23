document.querySelectorAll("[data-quiz]").forEach((quiz) => {
  const feedback = quiz.querySelector(".feedback");
  quiz.querySelectorAll("button[data-choice]").forEach((button) => {
    button.addEventListener("click", () => {
      const correct = button.dataset.choice === quiz.dataset.answer;
      feedback.textContent = correct
        ? "正确：领头车主动寻找并连接跟随车。"
        : "再想一下：等待被连接的一方才是从机。";
      feedback.style.color = correct ? "#176b4d" : "#9b5b00";
    });
  });
});
