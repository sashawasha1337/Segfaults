import React, { useEffect, useState } from "react";
import { useParams } from "react-router-dom";
import { db } from "../firebaseConfig";
import {
  doc,
  getDoc,
  updateDoc,
  arrayUnion,
  arrayRemove
} from "firebase/firestore";

export default function ManageUsersPage() {
  const { robotID: robotId } = useParams();
  const [robot, setRobot] = useState(null);
  const [newUserEmail, setNewUserEmail] = useState("");

  const norm = (s) => (s || "").trim().toLowerCase();

  useEffect(() => {
    if (!robotId) {
        console.error("No robotId in URL params!");
        return;
    }

    const loadRobot = async () => {
        try {
        const ref = doc(db, "robots", robotId);
        const snap = await getDoc(ref);
        if (snap.exists()) setRobot({ id: snap.id, ...snap.data() });
        } catch (err) {
        console.error("Error loading robot:", err);
        }
    };

    loadRobot();
    }, [robotId]);


  const handleAddUser = async () => {
    if (!newUserEmail) return;
    const email = norm(newUserEmail);

    const ref = doc(db, "robots", robotId);
    await updateDoc(ref, {
      users: arrayUnion(email)
    });

    setRobot((r) => ({ ...r, users: [...r.users, email] }));
    setNewUserEmail("");
  };

  const handleRemoveUser = async (email) => {
    const ref = doc(db, "robots", robotId);
    await updateDoc(ref, {
      users: arrayRemove(email)
    });

    setRobot((r) => ({ ...r, users: r.users.filter((u) => u !== email) }));
  };

  if (!robot) return <p>Loading...</p>;

  return (
    <div style={{ padding: 20 }}>
      <h2>Manage Users - {robot.name}</h2>

      <h3>Users</h3>
      {robot.users?.map((email) => (
        <div key={email} style={{ marginBottom: 10 }}>
          {email}
          <button
            style={{ marginLeft: 10 }}
            onClick={() => handleRemoveUser(email)}
          >
            Remove
          </button>
        </div>
      ))}

      <h3>Add User</h3>
      <input
        type="email"
        value={newUserEmail}
        onChange={(e) => setNewUserEmail(e.target.value)}
        placeholder="Enter user email"
      />
      <button onClick={handleAddUser}>Add</button>
    </div>
  );
}
