import { auth } from "./firebase.js";
import { signInWithEmailAndPassword, signOut, onAuthStateChanged } from "https://www.gstatic.com/firebasejs/12.19.0/firebase-auth.js";

// 認証状態の監視（ログインページ用）
export function watchAuthForLogin(onLoggedIn) {
  onAuthStateChanged(auth, (user) => {
    if (user) onLoggedIn();
  });
}

// 認証状態の監視（一覧・詳細ページ用）
export function requireAuth(onLoggedOut) {
  onAuthStateChanged(auth, (user) => {
    if (!user) onLoggedOut();
  });
}

// ログイン
export async function login(email, password) {
  await signInWithEmailAndPassword(auth, email, password);
}

// ログアウト
export async function logout() {
  await signOut(auth);
}
