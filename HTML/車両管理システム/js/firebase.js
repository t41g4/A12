import { initializeApp } from "https://www.gstatic.com/firebasejs/12.19.0/firebase-app.js";
import { getAuth } from "https://www.gstatic.com/firebasejs/12.19.0/firebase-auth.js";
import { getFirestore } from "https://www.gstatic.com/firebasejs/12.19.0/firebase-firestore.js";

const firebaseConfig = {
  apiKey: "AIzaSyCe_mL2eixG-h6hNQmI0OYzDeFvymFXtVI",
  authDomain: "cycle-database-2c708.firebaseapp.com",
  projectId: "cycle-database-2c708",
  storageBucket: "cycle-database-2c708.firebasestorage.app",
  messagingSenderId: "335631921023",
  appId: "1:335631921023:web:648d4dd134b4c5a6bdb44a",
  measurementId: "G-HN54KJGEGL"
};

const app = initializeApp(firebaseConfig);
export const auth = getAuth(app);
export const db = getFirestore(app);
